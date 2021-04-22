#ifndef CAMERACMD_H
#define CAMERACMD_H
using namespace std;
enum ImageType {
    DEPTH = 1,
    COLOR = 2,
    MatXYZ = 16,
    COLOR_DEPTH = COLOR | DEPTH,
};

namespace Command {
    std::string CaptureImage = "CaptureImage";
    std::string GetCameraIntri = "GetCameraIntri";
    std::string GetCameraId = "GetCameraId";
    std::string GetCameraInfo = "GetCameraInfo";
    std::string GetCamera2dInfo = "GetCamera2dInfo";
    std::string GetServerInfo = "GetServerInfo";
    std::string SetCameraParams = "SetCameraConfig";
    std::string GetCameraParams = "GetCameraConfig";
    std::string CaptureGratingImage = "CaptureGratingImage";
    std::string GetImageFormat = "GetImageFormat";
}

namespace Service {
    std::string cmd = "cmd";
    std::string property_name = "property_name";
    std::string property_value = "property_value";
    std::string image_type = "image_type";
    std::string persistent = "persistent";
    std::string camera_config = "camera_config";
    std::string image_format = "image_format";
    std::string size2d = "imageSize2D";
    std::string size3d = "imageSize3D";
}

constexpr int Encode32FBias = 32768;
const int SIZE_OF_JSON = 4;
const int SIZE_OF_SCALE = 8;

#endif // CAMERACMD_H
