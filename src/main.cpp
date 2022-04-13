#include "MechEyeApi.h"
#include <opencv2/imgcodecs.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

bool isNumber(const std::string& str);
void showError(const mmind::api::ErrorStatus& status);
void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo);

class MechMindCamera
{
public:
    mmind::api::MechEyeDevice device;
    ros::NodeHandle nh;
    mmind::api::DeviceIntri intri;

    ros::Publisher pub_color;
    ros::Publisher pub_depth;
    ros::Publisher pub_pcl;
    ros::Publisher pub_pcl_color;
    ros::Publisher pub_camera_info;

    ros::ServiceServer service;
    std::string camera_ip;
    bool save_file = false;
    bool use_external_intri = false;
    double fx = 0;
    double fy = 0;
    double u = 0;
    double v = 0;

    MechMindCamera()
    {
        ros::NodeHandle pnh("~");
        // Camera ip should be modified to actual ip address.
        pnh.getParam("camera_ip", camera_ip);
        pnh.getParam("save_file", save_file);
        pnh.getParam("use_external_intri", use_external_intri);
        pnh.getParam("fx", fx);
        pnh.getParam("fy", fy);
        pnh.getParam("u", u);
        pnh.getParam("v", v);

        pub_color = nh.advertise<sensor_msgs::Image>("/mechmind/color_image", 1, true);
        pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind/depth_image", 1, true);
        pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/point_cloud", 1, true);
        pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/color_point_cloud", 1, true);
        pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/mechmind/camera_info", 1, true);

        std::cout << "Find Mech-Eye device :" << std::endl;

        std::vector<mmind::api::MechEyeDeviceInfo> deviceInfoList =
        mmind::api::MechEyeDevice::enumerateMechEyeDeviceList(); if (deviceInfoList.empty())
        {
            std::cout << "No Mech-Eye device found." << std::endl;
            return;
        }

        for (int i = 0; i < deviceInfoList.size(); i++)
        {
            std::cout << "Mech-Eye device index : " << i << std::endl;
            printDeviceInfo(deviceInfoList[i]);
        }

        std::cout << "Please enter the device index you want to connect: ";
        unsigned inputIndex;

        while (1)
        {
            std::string str;
            std::cin >> str;
            if (isNumber(str) && atoi(str.c_str()) < deviceInfoList.size())
            {
                inputIndex = atoi(str.c_str());
                break;
            }
            std::cout << "Input invalid! Please enter the device index you want to connect: ";
        }

        mmind::api::ErrorStatus status;
        mmind::api::MechEyeDevice device;
        status = device.connect(deviceInfoList[inputIndex]);

        // Uncomment the following lines to connect a camera with ip inside .launch file

        // mmind::api::ErrorStatus status;
        // mmind::api::MechEyeDeviceInfo info;
        // info.firmwareVersion = "1.5.0";
        // info.ipAddress = camera_ip;
        // info.port = 5577;
        // status = device.connect(info);

        if (!status.isOK())
        {
            showError(status);
            return;
        }

        std::cout << "Connect Mech-Eye Success." << std::endl;

        mmind::api::MechEyeDeviceInfo deviceInfo;
        status = device.getDeviceInfo(deviceInfo);
        printDeviceInfo(deviceInfo);

        if (use_external_intri)
        {
            intri.cameraMatrix[0] = fx;
            intri.cameraMatrix[1] = fy;
            intri.cameraMatrix[2] = u;
            intri.cameraMatrix[3] = v;
        }
        else
        {
            showError(device.getDeviceIntri(intri));
        }

        service = nh.advertiseService("run_mechmind_camera", &MechMindCamera::get_camera_callback, this);
        while (ros::ok())
        {
            pub_rgb_image();
        }
    }

    void pub_rgb_image()
    {
        mmind::api::ColorMap colorMap;
        showError(device.captureColorMap(colorMap));
        cv::Mat color = cv::Mat(colorMap.height(), colorMap.width(), CV_8UC3, colorMap.data());
        cv_bridge::CvImage cv_image;
        cv_image.image = color;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        ros_image.header.frame_id = "mechmind_camera";
        ros_image.header.stamp = ros::Time::now();
        pub_color.publish(ros_image);

        // publish camera info
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.stamp = ros_image.header.stamp;
        camera_info.header.frame_id = "mechmind_camera";
        camera_info.height = color.rows;
        camera_info.width = color.cols;
        camera_info.distortion_model = "plumb_bob";

        camera_info.D = std::vector<double>(intri.distCoeffs, intri.distCoeffs + 5);

        std::vector<double> K{ intri.cameraMatrix[0],
                               0.0,
                               intri.cameraMatrix[2],
                               0.0,
                               intri.cameraMatrix[1],
                               intri.cameraMatrix[3],
                               0.0,
                               0.0,
                               1.0 };
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info.K[i] = K[i];
        }

        std::vector<double> R{ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info.R[i] = R[i];
        }

        std::vector<double> P{ intri.cameraMatrix[0],
                               0.0,
                               intri.cameraMatrix[2],
                               0.0,
                               0.0,
                               intri.cameraMatrix[1],
                               intri.cameraMatrix[3],
                               0.0,
                               0.0,
                               0.0,
                               1.0,
                               0.0 };
        for (size_t i = 0; i < 12; ++i)
        {
            camera_info.P[i] = P[i];
        }
        pub_camera_info.publish(camera_info);
    }

    bool get_camera_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        mmind::api::DepthMap depthMap;
        showError(device.captureDepthMap(depthMap));
        cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());

        mmind::api::PointXYZMap pointXYZMap;
        showError(device.capturePointXYZMap(pointXYZMap));
        pcl::PointCloud<pcl::PointXYZ> cloud(pointXYZMap.width(), pointXYZMap.height());
        uint32_t size = pointXYZMap.height() * pointXYZMap.width();
        cloud.resize(size);

        for (size_t i = 0; i < size; i++)
        {
            cloud[i].x = 0.001 * pointXYZMap[i].x;
            cloud[i].y = 0.001 * pointXYZMap[i].y;
            cloud[i].z = 0.001 * pointXYZMap[i].z;
        }

        mmind::api::ColorMap colorMap;
        showError(device.captureColorMap(colorMap));
        pcl::PointCloud<pcl::PointXYZRGB> color_cloud(pointXYZMap.width(), pointXYZMap.height());
        color_cloud.resize(size);

        for (size_t i = 0; i < size; ++i)
        {
            color_cloud[i].x = 0.001 * pointXYZMap[i].x;
            color_cloud[i].y = 0.001 * pointXYZMap[i].y;
            color_cloud[i].z = 0.001 * pointXYZMap[i].z;

            color_cloud[i].r = colorMap[i].r;
            color_cloud[i].g = colorMap[i].g;
            color_cloud[i].b = colorMap[i].b;
        }

        cv_bridge::CvImage cv_depth;
        cv_depth.image = depth;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        sensor_msgs::Image ros_depth;
        sensor_msgs::PointCloud2 ros_cloud;
        sensor_msgs::PointCloud2 ros_color_cloud;
        cv_depth.toImageMsg(ros_depth);
        pcl::toROSMsg(cloud, ros_cloud);
        pcl::toROSMsg(color_cloud, ros_color_cloud);
        ros_depth.header.frame_id = "mechmind_camera";
        ros_depth.header.stamp = ros::Time::now();
        ros_cloud.header.frame_id = "mechmind_camera";
        ros_cloud.header.stamp = ros::Time::now();
        ros_color_cloud.header.frame_id = "mechmind_camera";
        ros_color_cloud.header.stamp = ros::Time::now();

        pub_pcl.publish(ros_cloud);
        pub_depth.publish(ros_depth);
        pub_pcl_color.publish(ros_color_cloud);

        if (save_file)
        {
            cv::imwrite("/tmp/mechmind_depth.png", depth);
            pcl::PLYWriter().write("/tmp/mechmind_color_cloud.ply", color_cloud);
            pcl::PLYWriter().write("/tmp/mechmind_cloud.ply", cloud);
        }
        res.success = true;
        return true;
    }

    ~MechMindCamera() = default;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechmind_camera_publisher_service");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    MechMindCamera mm_camera;

    ros::waitForShutdown();
    return 0;
}

void showError(const mmind::api::ErrorStatus& status)
{
    if (status.isOK())
        return;
    std::cout << "Error Code : " << status.errorCode << ", Error Description: " << status.errorDescription << "."
              << std::endl;
}

void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo)
{
    std::cout << "............................" << std::endl;
    std::cout << "Camera Model Name: " << deviceInfo.model << std::endl;
    std::cout << "Camera ID:         " << deviceInfo.id << std::endl;
    std::cout << "Camera IP Address: " << deviceInfo.ipAddress << std::endl;
    std::cout << "Hardware Version:  "
              << "V" << deviceInfo.hardwareVersion << std::endl;
    std::cout << "Firmware Version:  "
              << "V" << deviceInfo.firmwareVersion << std::endl;
    std::cout << "............................" << std::endl;
    std::cout << std::endl;
}

bool isNumber(const std::string& str)
{
    for (auto it = str.cbegin(); it != str.cend(); ++it)
    {
        if (*it < '0' || *it > '9')
            return false;
    }
    return true;
}