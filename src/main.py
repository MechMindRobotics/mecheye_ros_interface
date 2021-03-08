#!/usr/bin/env python
from CameraClient import CameraClient, ImageType, Command, CameraIntri
import sys
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os

SAVE_PATH = "/home/dominique/tmp"

if __name__ == '__main__':
    rospy.init_node('mech_eye')
    camera = CameraClient()
    save_file = True
    # camera ip should be modified to actual ip address
    camera_ip = "192.168.3.180"
    rospy.get_param("camera_ip", camera_ip)
    rospy.get_param("save_file", save_file)

    if not camera.connect(camera_ip):
        exit(-1)
    intri = camera.getCameraIntri()
    print ("Camera Info: %s" % (camera.getCameraInfo()))
    print ("Camera ID: %s" % (camera.getCameraId()))
    print ("Version: %s" % (camera.getCameraVersion()))

    pub_color = rospy.Publisher('/color_image', Image, queue_size=1)
    pub_depth = rospy.Publisher('/depth_image', Image, queue_size=1)

    loop_rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        depth = camera.captureDepthImg()
        color = camera.captureColorImg()

        if len(depth) == 0 or len(color) == 0:
            exit(-2)
        ros_image = CvBridge().cv2_to_imgmsg(color, "bgr8")
        ros_depth = CvBridge().cv2_to_imgmsg(depth, "64FC1")

        ros_image.header.frame_id = "mechmind_camera"
        ros_depth.header.frame_id = "mechmind_camera"

        pub_color.publish(ros_image)
        pub_depth.publish(ros_depth)

        if save_file:
            if not os.path.exists(SAVE_PATH):
                os.makedirs(SAVE_PATH)
            cv2.imwrite(SAVE_PATH + "/mechmind_depth.png", depth)
            cv2.imwrite(SAVE_PATH + "/mechmind_color.jpg", color)

        loop_rate.sleep()

    exit(0)
