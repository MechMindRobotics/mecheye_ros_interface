#pragma once
#include <area_scan_3d_camera/Camera.h>
#include <area_scan_3d_camera/CameraProperties.h>
#include <area_scan_3d_camera/Frame3D.h>
#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <mecheye_ros_interface/CaptureColorImage.h>
#include <mecheye_ros_interface/CaptureStereoColorImages.h>
#include <mecheye_ros_interface/CaptureTexturedPointCloud.h>
#include <mecheye_ros_interface/CaptureDepthMap.h>
#include <mecheye_ros_interface/CapturePointCloud.h>
#include <mecheye_ros_interface/DeleteUserSet.h>
#include <mecheye_ros_interface/DeviceInfo.h>
#include <mecheye_ros_interface/AddUserSet.h>
#include <mecheye_ros_interface/GetAllUserSets.h>
#include <mecheye_ros_interface/GetCurrentUserSet.h>
#include <mecheye_ros_interface/SaveAllSettingsToUserSets.h>
#include <mecheye_ros_interface/SetCurrentUserSet.h>
#include <mecheye_ros_interface/SetIntParameter.h>
#include <mecheye_ros_interface/GetIntParameter.h>
#include <mecheye_ros_interface/SetBoolParameter.h>
#include <mecheye_ros_interface/GetBoolParameter.h>
#include <mecheye_ros_interface/SetFloatParameter.h>
#include <mecheye_ros_interface/GetFloatParameter.h>
#include <mecheye_ros_interface/SetEnumParameter.h>
#include <mecheye_ros_interface/GetEnumParameter.h>
#include <mecheye_ros_interface/SetRangeParameter.h>
#include <mecheye_ros_interface/GetRangeParameter.h>
#include <mecheye_ros_interface/SetROIParameter.h>
#include <mecheye_ros_interface/GetROIParameter.h>
#include <mecheye_ros_interface/SetFloatArrayParameter.h>
#include <mecheye_ros_interface/GetFloatArrayParameter.h>

class MechMindCamera
{
public:
    MechMindCamera();

private:
    ros::NodeHandle nh;
    mmind::eye::Camera camera;
    mmind::eye::CameraIntrinsics intrinsics;

    std::string camera_ip;
    bool save_file = false;
    bool use_external_intri = false;
    double fx = 0;
    double fy = 0;
    double u = 0;
    double v = 0;

    ros::Publisher pub_color;
    ros::Publisher pub_color_left;
    ros::Publisher pub_color_right;
    ros::Publisher pub_depth;
    ros::Publisher pub_pcl;
    ros::Publisher pub_pcl_color;
    ros::Publisher pub_camera_info;

    void publishColorMap(mmind::eye::Color2DImage& color2DImage);
    void publishStereoColorMap(mmind::eye::Color2DImage& leftColor2DImage,
                               mmind::eye::Color2DImage& rightColor2DImage);
    void publishDepthMap(mmind::eye::DepthMap& depthMap);
    void publishPointCloud(mmind::eye::PointCloud& pointCloud);
    void publishColorPointCloud(mmind::eye::TexturedPointCloud& texturedPointCloud);
    void publishColorCameraInfo(const std_msgs::Header& header, int width, int height);
    void publishDepthCameraInfo(const std_msgs::Header& header, int width, int height);

    ros::ServiceServer add_user_set_service;
    ros::ServiceServer capture_color_image_service;
    ros::ServiceServer capture_stereo_color_images_service;
    ros::ServiceServer capture_textured_point_cloud_service;
    ros::ServiceServer capture_depth_map_service;
    ros::ServiceServer capture_point_cloud_service;

    ros::ServiceServer delete_user_set_service;
    ros::ServiceServer device_info_service;
    ros::ServiceServer get_all_user_sets_service;
    ros::ServiceServer get_current_user_set_service;
    ros::ServiceServer save_all_settings_to_user_sets_service;
    ros::ServiceServer set_current_user_set_service;

    ros::ServiceServer get_float_sequence_parameter_service;
    ros::ServiceServer set_float_sequence_parameter_service;
    ros::ServiceServer get_roi_parameter_service;
    ros::ServiceServer set_roi_parameter_service;
    ros::ServiceServer get_range_parameter_service;
    ros::ServiceServer set_range_parameter_service;
    ros::ServiceServer get_int_parameter_service;
    ros::ServiceServer set_int_parameter_service;
    ros::ServiceServer get_bool_parameter_service;
    ros::ServiceServer set_bool_parameter_service;
    ros::ServiceServer get_enum_parameter_service;
    ros::ServiceServer set_enum_parameter_service;
    ros::ServiceServer get_float_parameter_service;
    ros::ServiceServer set_float_parameter_service;

    bool capture_color_image_callback(mecheye_ros_interface::CaptureColorImage::Request& req,
                                      mecheye_ros_interface::CaptureColorImage::Response& res);

    bool capture_textured_point_cloud_callback(
        mecheye_ros_interface::CaptureTexturedPointCloud::Request& req,
        mecheye_ros_interface::CaptureTexturedPointCloud::Response& res);
    bool capture_depth_map_callback(mecheye_ros_interface::CaptureDepthMap::Request& req,
                                    mecheye_ros_interface::CaptureDepthMap::Response& res);
    bool capture_point_cloud_callback(mecheye_ros_interface::CapturePointCloud::Request& req,
                                      mecheye_ros_interface::CapturePointCloud::Response& res);
    bool capture_stereo_color_images_callback(
        mecheye_ros_interface::CaptureStereoColorImages::Request& req,
        mecheye_ros_interface::CaptureStereoColorImages::Response& res);
    bool add_user_set_callback(mecheye_ros_interface::AddUserSet::Request& req,
                               mecheye_ros_interface::AddUserSet::Response& res);
    bool delete_user_set_callback(mecheye_ros_interface::DeleteUserSet::Request& req,
                                  mecheye_ros_interface::DeleteUserSet::Response& res);
    bool device_info_callback(mecheye_ros_interface::DeviceInfo::Request& req,
                              mecheye_ros_interface::DeviceInfo::Response& res);
    bool get_all_user_sets_callback(mecheye_ros_interface::GetAllUserSets::Request& req,
                                    mecheye_ros_interface::GetAllUserSets::Response& res);
    bool get_current_user_set_callback(mecheye_ros_interface::GetCurrentUserSet::Request& req,
                                       mecheye_ros_interface::GetCurrentUserSet::Response& res);
    bool save_all_settings_to_user_sets_callback(
        mecheye_ros_interface::SaveAllSettingsToUserSets::Request& req,
        mecheye_ros_interface::SaveAllSettingsToUserSets::Response& res);

    bool set_current_user_set_callback(mecheye_ros_interface::SetCurrentUserSet::Request& req,
                                       mecheye_ros_interface::SetCurrentUserSet::Response& res);

    bool get_float_array_parameter_callback(
        mecheye_ros_interface::GetFloatArrayParameter::Request& req,
        mecheye_ros_interface::GetFloatArrayParameter::Response& res);

    bool set_float_array_parameter_callback(
        mecheye_ros_interface::SetFloatArrayParameter::Request& req,
        mecheye_ros_interface::SetFloatArrayParameter::Response& res);

    bool get_roi_parameter_callback(mecheye_ros_interface::GetROIParameter::Request& req,
                                    mecheye_ros_interface::GetROIParameter::Response& res);

    bool set_roi_parameter_callback(mecheye_ros_interface::SetROIParameter::Request& req,
                                    mecheye_ros_interface::SetROIParameter::Response& res);

    bool get_range_parameter_callback(mecheye_ros_interface::GetRangeParameter::Request& req,
                                      mecheye_ros_interface::GetRangeParameter::Response& res);

    bool set_range_parameter_callback(mecheye_ros_interface::SetRangeParameter::Request& req,
                                      mecheye_ros_interface::SetRangeParameter::Response& res);

    bool get_int_parameter_callback(mecheye_ros_interface::GetIntParameter::Request& req,
                                    mecheye_ros_interface::GetIntParameter::Response& res);

    bool set_int_parameter_callback(mecheye_ros_interface::SetIntParameter::Request& req,
                                    mecheye_ros_interface::SetIntParameter::Response& res);

    bool get_bool_parameter_callback(mecheye_ros_interface::GetBoolParameter::Request& req,
                                     mecheye_ros_interface::GetBoolParameter::Response& res);

    bool set_bool_parameter_callback(mecheye_ros_interface::SetBoolParameter::Request& req,
                                     mecheye_ros_interface::SetBoolParameter::Response& res);

    bool get_enum_parameter_callback(mecheye_ros_interface::GetEnumParameter::Request& req,
                                     mecheye_ros_interface::GetEnumParameter::Response& res);

    bool set_enum_parameter_callback(mecheye_ros_interface::SetEnumParameter::Request& req,
                                     mecheye_ros_interface::SetEnumParameter::Response& res);

    bool get_float_parameter_callback(mecheye_ros_interface::GetFloatParameter::Request& req,
                                      mecheye_ros_interface::GetFloatParameter::Response& res);

    bool set_float_parameter_callback(mecheye_ros_interface::SetFloatParameter::Request& req,
                                      mecheye_ros_interface::SetFloatParameter::Response& res);
};
