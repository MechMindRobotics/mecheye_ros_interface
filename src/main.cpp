#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mechmind_camera_publisher");
    ros::NodeHandle nh("~");
    CameraClient camera;
    bool save_file = false;
    // Camera ip should be modified to actual ip address.
    std::string camera_ip = "192.168.1.240";
    nh.getParam("camera_ip", camera_ip);
    nh.getParam("save_file", save_file);

    if (!camera.setIp(camera_ip)) return -1;
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl;
    CameraIntri intri = camera.getCameraIntri();
	
    ros::Publisher pub_color = nh.advertise<sensor_msgs::Image>("/color_image", 1);
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image>("/depth_image", 1);
    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud", 1);
    ros::Publisher pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2> ("/color_point_cloud", 1);

    ros::Rate loop_rate(5);

    while (nh.ok()) 
    {

    const cv::Mat depth = camera.captureDepthImg();
    const cv::Mat color = camera.captureColorImg();
    if (depth.empty() || color.empty()) return -2;

    const pcl::PointCloud<pcl::PointXYZ> cloud = camera.capturePointCloud(intri);

    cv_bridge::CvImage cv_image, cv_depth;
    cv_image.image = color;
    cv_depth.image = depth;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::Image ros_image;
    sensor_msgs::Image ros_depth;
    sensor_msgs::PointCloud2 ros_cloud;
    sensor_msgs::PointCloud2 ros_color_cloud;

    cv_image.toImageMsg(ros_image);
    cv_depth.toImageMsg(ros_depth);
    pcl::toROSMsg(cloud, ros_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud = PointCloudTools::getColoredCloud(color, depth, intri);
    pcl::toROSMsg(color_cloud, ros_color_cloud);
    ros_image.header.frame_id = "mechmind_camera";
    ros_depth.header.frame_id = "mechmind_camera";
    ros_cloud.header.frame_id = "mechmind_camera";
    ros_color_cloud.header.frame_id = "mechmind_camera";

    pub_pcl.publish(ros_cloud);
    pub_color.publish(ros_image);
    pub_depth.publish(ros_depth);
    pub_pcl_color.publish(ros_color_cloud);

    if (save_file){
        cv::imwrite("/tmp/mechmind_depth.png", depth);
        cv::imwrite("/tmp/mechmind_color.jpg", color);
        PointCloudTools::savePointCloud("/tmp/mechmind_color_cloud.ply", color_cloud);
        PointCloudTools::savePointCloud("/tmp/mechmind_cloud.ply", cloud);
        }
    loop_rate.sleep();
    }
    return 0;
}
