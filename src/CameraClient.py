#!/usr/bin/env python
from ZmqClient import ZmqClient
import sys
import cv2
import numpy as np
import open3d
import ctypes
from struct import unpack
import json

Encode32FBias = 32768
SIZE_OF_DOUBLE = ctypes.sizeof(ctypes.c_double)
SIZE_OF_INT = ctypes.sizeof(ctypes.c_int32)
SIZE_OF_JSON = 4
SIZE_OF_SCALE = 8
VISIBLE_DEPTH_THRESHOLD = 1e-3

class ImageType():
    DEPTH = 1
    COLOR = 2
    COLOR_DEPTH = COLOR | DEPTH


class Service():
    cmd = "cmd"
    property_name = "property_name"
    property_value = "property_value"
    image_type = "image_type"
    persistent = "persistent"
    camera_config = "camera_config"

class Command:
    CaptureImage = "CaptureImage"
    
    GetCameraIntri = "GetCameraIntri"
    GetCameraId = "GetCameraId"
    GetCameraInfo = "GetCameraInfo"
    GetCamera2dInfo = "GetCamera2dInfo"
    GetServerInfo = "GetServerInfo"
    SetCameraParams = "SetCameraConfig"
    GetCameraParams = "GetCameraConfig"
    CaptureGratingImage = "CaptureGratingImage"


class CameraIntri:
    def __init__(self):
        self.__fx = 0.0
        self.__fy = 0.0
        self.__u = 0.0
        self.__v = 0.0

    def isZero(self):
        return (self.__fx == 0.0) and (self.__fy == 0.0) and (self.__u == 0.0) and (self.__v == 0.0)

    def setValue(self, fx, fy, u, v):
        self.__fx = fx
        self.__fy = fy
        self.__u = u
        self.__v = v

    def getValue(self):
        return self.__fx, self.__fy, self.__u, self.__v


def readDouble(data, pos):
    if pos + SIZE_OF_DOUBLE > len(data):
        return 0
    strFromQDataStream = data[pos: pos + SIZE_OF_DOUBLE]
    v = unpack(">d", strFromQDataStream)
    return v[0]

def readInt(data,pos):
    if pos + SIZE_OF_INT > len(data):
        return 0
    strFromQDataStream = data[pos: pos + SIZE_OF_INT]
    v = unpack(">i",strFromQDataStream)
    return v[0]

def asMat(data, offset=0):
    mat = []
    for i in range(offset, len(data)):
        mat.append(data[i]) # in Python 2.x you may need to unpack data: unpack(">B",data[i])
    mat = np.asarray(mat, dtype="uint8")
    return mat

def matC1ToC3(mat):
    if len(mat) == 0:
        return []
    if len(np.shape(mat)) != 2 or np.shape(mat)[0]%3 != 0:
        return []
    rows, cols = np.shape(mat)
    rel = []
    rel.append(mat[0:int(rows/3),:])
    rel.append(mat[int(rows/3):int(2*rows/3),:])
    rel.append(mat[int(2*rows/3):rows,:])
    rel = cv2.merge(rel)
    return rel


def read32FC3Mat(data,scale):
    if len(data) == 0:
        return []
    matC1 = cv2.imdecode(asMat(data),cv2.IMREAD_ANYDEPTH)
    bias16UC3 = matC1ToC3(matC1)
    t = np.float32(bias16UC3)
    mat32F = (t - Encode32FBias)/scale
    return mat32F

def read32FC1Mat(data, scale):
    if len(data) == 0:
        return []
    bias16U = cv2.imdecode(asMat(data), cv2.IMREAD_ANYDEPTH)
    bias32F = bias16U.astype(np.float32)
    mat32F = bias32F
    mat32F[:, :] -= Encode32FBias
    if scale == 0:
        return cv2.Mat()
    else:
        return mat32F / scale


class CameraClient(ZmqClient):
    __kImagePort = 5577

    def __init__(self):
        ZmqClient.__init__(self)

    def connect(self, ip):
        return ZmqClient.setAddr(self, ip, self.__kImagePort, 60000)

    def captureDepthImg(self):
        response = self.__sendRequest(Command.CaptureImage, image_type = ImageType.DEPTH)
        jsonSize = readInt(response, 0)
        scale = readDouble(response, jsonSize + SIZE_OF_JSON)
        imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE)
        imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + SIZE_OF_INT
        imageDepth = response[imageBegin: imageBegin + imageSize]
        if len(imageDepth) == 0:
            print("Client depth image is empty!")
            return {}
        print("Depth image captured!")
        return read32FC1Mat(imageDepth, scale)

    def captureColorImg(self):
        response = self.__sendRequest(Command.CaptureImage, image_type = ImageType.COLOR)
        jsonSize = readInt(response,0)
        imageSize = readInt(response,SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE)
        imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + SIZE_OF_INT
        imageRGB = response[imageBegin : imageBegin + imageSize]
        if len(imageRGB) == 0:
            print("Client depth image is empty!")
            return {}
        print("Color image captured!")
        return cv2.imdecode(asMat(imageRGB), cv2.IMREAD_COLOR)

    def getCameraIntri(self):
        response = self.__sendRequest(Command.GetCameraIntri, 0)
        print("Camera intrinsics: ")
        intriJson = json.loads(response[SIZE_OF_JSON:-1])
        intriValue = intriJson["camera_intri"]['intrinsic']

        intri = CameraIntri()
        intri.setValue(
            float(
                intriValue[0]), float(
                intriValue[1]), float(
                intriValue[2]), float(
                    intriValue[3]))

        intriValueDouble = intri.getValue()
        print("fx = %.13f" % (intriValueDouble[0]))
        print("fy = %.13f" % (intriValueDouble[1]))
        print("u = %.13f" % (intriValueDouble[2]))
        print("v = %.13f" % (intriValueDouble[3]))
        return intriValueDouble

    def getCameraId(self):
        return self.getCameraInfo()["eyeId"]

    def getCameraInfo(self):
        response = self.__sendRequest(Command.GetCameraInfo)
        info = json.loads(response[SIZE_OF_JSON:-1])
        return info["camera_info"]

    def getCameraVersion(self):
        return self.getCameraInfo()["version"]

    def getParameter(self,paraName):
        request = {}
        request[Service.cmd] = Command.GetCameraParams
        request[Service.property_name] = paraName
        request = json.dumps(request)
        reply = ZmqClient.sendReq(self, request)
        reply = json.loads(reply[SIZE_OF_JSON:-1])
        allConfigs =  reply["camera_config"]["configs"][0]
        if(paraName in allConfigs):
            return allConfigs[paraName]
        print("Property" + paraName + "not exist!")
        return None

    def setParameter(self,paraName,value):
        request = {}
        request[Service.cmd] = Command.SetCameraParams
        request[Service.camera_config] = {}
        request[Service.camera_config][paraName] = value
        request[Service.persistent] = "false"
        request = json.dumps(request)
        reply = ZmqClient.sendReq(self, request)
        reply = json.loads(reply[SIZE_OF_JSON:-1])
        if ("err_msg" in reply):
            return reply["err_msg"]

    def __sendRequest(self, commandx, property_name = "", value = 0, image_type = 0):
        request = {}
        request[Service.cmd] = commandx
        request[Service.property_name] = property_name
        request[Service.property_value] = value
        request[Service.image_type] = image_type
        request = json.dumps(request)
        reply = ZmqClient.sendReq(self, request)
        return reply

    def captureCloud(self):
        response = self.__sendRequest(Command.CaptureImage,image_type=ImageType.MatXYZ)
        jsonSize = readInt(response, 0)
        scale = readDouble(response, jsonSize + SIZE_OF_JSON)
        imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE)
        imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + SIZE_OF_INT
        imageDepth = response[imageBegin: imageBegin + imageSize]
        depthC3 = read32FC3Mat(imageDepth,scale)
        color = self.captureColorImg()
        return self.getRGBCloud(color, depthC3)
    
    def removeZero(self, depth, color):
        nonZeroIndices = depth[:,2] > VISIBLE_DEPTH_THRESHOLD
        return depth[nonZeroIndices], color[nonZeroIndices]

    def getRGBCloud(self,color,depth):
        test_pcd = open3d.geometry.PointCloud()  # 定义点云
        color.flatten()
        color = color / 256
        color.resize(int(np.size(color)/3),3)
        depth.flatten()
        depth = depth * 0.001
        depth.resize(int(np.size(depth)/3),3)
        depth, color = self.removeZero(depth, color)
        test_pcd.points = open3d.utility.Vector3dVector(depth)  # 定义点云坐标位置
        test_pcd.colors = open3d.utility.Vector3dVector(color)  # 定义点云的颜色
        return test_pcd
