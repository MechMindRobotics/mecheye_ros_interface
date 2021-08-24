#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MechMindCamera {
public:
    CameraClient camera;
    ros::NodeHandle nh;
    CameraIntri intri;

    ros::Publisher pub_color;
    ros::Publisher pub_depth;
    ros::Publisher pub_pcl;
    ros::Publisher pub_pcl_color;
    ros::Publisher pub_camera_info;

    ros::ServiceServer service;
    std::string camera_ip;
    bool save_file=false;

    MechMindCamera() {
        ros::NodeHandle pnh("~");
        pnh.getParam("camera_ip", camera_ip);
        pnh.getParam("save_file", save_file);
        // Camera ip should be modified to actual ip address.

        pub_color = nh.advertise<sensor_msgs::Image>("/mechmind/color_image",1, true);
        pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind/depth_image", 1, true);
        pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/point_cloud", 1, true);
        pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/color_point_cloud", 1, true);
        pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/mechmind/camera_info", 1, true);
        if (!camera.connect(camera_ip)) return;
        std::cout << "Camera ID: " << camera.getCameraId() << std::endl;
        std::cout << "Version: " << camera.getCameraVersion() << std::endl;
        std::cout << "Color Image Size: " << camera.getColorImgSize() << std::endl;
        std::cout << "Depth Image Size: " << camera.getDepthImgSize() << std::endl;
        intri = camera.getCameraIntri();
        service = nh.advertiseService("run_mechmind_camera",
                                      &MechMindCamera::get_camera_callback, this);
    }


    bool get_camera_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        const cv::Mat depth = camera.captureDepthImg();
        const cv::Mat color = camera.captureColorImg();
        if (depth.empty() || color.empty()) return -2;

        const pcl::PointCloud<pcl::PointXYZ> cloud = camera.capturePointCloud();

        cv_bridge::CvImage cv_image, cv_depth;
        cv_image.image = color;
        cv_depth.image = depth;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        sensor_msgs::Image ros_image;
        sensor_msgs::Image ros_depth;
        sensor_msgs::PointCloud2 ros_cloud;
        sensor_msgs::PointCloud2 ros_color_cloud;

        // get camera info
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.stamp = ros::Time::now();
        camera_info.header.frame_id = "mechmind_camera";
        camera_info.height = depth.rows;
        camera_info.width = depth.cols;
        camera_info.distortion_model = "plumb_bob";
        std::vector<double> D(5, 0.0);
        camera_info.D = D;

        std::vector<double> K{intri.fx, 0.0, intri.u, 0.0, intri.fy, intri.v, 0.0, 0.0, 1.0};
        for(size_t i = 0; i < 9; ++i)
        {
            camera_info.K[i] = K[i];
        }

        std::vector<double> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        for(size_t i = 0; i < 9; ++i)
        {
            camera_info.R[i] = R[i];
        }

        std::vector<double> P{intri.fx, 0.0, intri.u, 0.0, 0.0, intri.fy, intri.v, 0.0, 0.0, 0.0, 1.0, 0.0};
        for(size_t i = 0; i < 12; ++i)
        {
            camera_info.P[i] = P[i];
        }

        cv_image.toImageMsg(ros_image);
        cv_depth.toImageMsg(ros_depth);
        pcl::toROSMsg(cloud, ros_cloud);

        pcl::PointCloud<pcl::PointXYZRGB> color_cloud = camera.captureRgbPointCloud();
        pcl::toROSMsg(color_cloud, ros_color_cloud);
        ros_image.header.frame_id = "mechmind_camera";
        ros_image.header.stamp = camera_info.header.stamp;
        ros_depth.header.frame_id = "mechmind_camera";
        ros_depth.header.stamp = camera_info.header.stamp;
        ros_cloud.header.frame_id = "mechmind_camera";
        ros_cloud.header.stamp = camera_info.header.stamp;
        ros_color_cloud.header.frame_id = "mechmind_camera";
        ros_color_cloud.header.stamp = camera_info.header.stamp;

        pub_pcl.publish(ros_cloud);
        pub_color.publish(ros_image);
        pub_depth.publish(ros_depth);
        pub_pcl_color.publish(ros_color_cloud);
        pub_camera_info.publish(camera_info);

        if (save_file){
            cv::imwrite("/tmp/mechmind_depth.png", depth);
            cv::imwrite("/tmp/mechmind_color.jpg", color);
            PointCloudTools::saveRgbPointCloud("/tmp/mechmind_color_cloud.ply", color_cloud);
            PointCloudTools::savePointCloud("/tmp/mechmind_cloud.ply", cloud);
        }
        res.success = true;
        return true;
    }

    ~MechMindCamera() = default;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mechmind_camera_publisher_service");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    MechMindCamera mm_camera;

    ros::waitForShutdown();
    return 0;
}
