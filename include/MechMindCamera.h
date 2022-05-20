#include <MechEyeApi.h>
#include <ros/ros.h>
#include <mecheye_ros_interface/AddUserSet.h>
#include <mecheye_ros_interface/CaptureColorMap.h>
#include <mecheye_ros_interface/CaptureColorPointCloud.h>
#include <mecheye_ros_interface/CaptureDepthMap.h>
#include <mecheye_ros_interface/CapturePointCloud.h>
#include <mecheye_ros_interface/DeleteUserSet.h>
#include <mecheye_ros_interface/DeviceInfo.h>
#include <mecheye_ros_interface/Get2DExpectedGrayValue.h>
#include <mecheye_ros_interface/Get2DExposureMode.h>
#include <mecheye_ros_interface/Get2DExposureSequence.h>
#include <mecheye_ros_interface/Get2DExposureTime.h>
#include <mecheye_ros_interface/Get2DROI.h>
#include <mecheye_ros_interface/Get2DSharpenFactor.h>
#include <mecheye_ros_interface/Get2DToneMappingEnable.h>
#include <mecheye_ros_interface/Get3DExposure.h>
#include <mecheye_ros_interface/Get3DGain.h>
#include <mecheye_ros_interface/Get3DROI.h>
#include <mecheye_ros_interface/GetAllUserSets.h>
#include <mecheye_ros_interface/GetCloudOutlierFilterMode.h>
#include <mecheye_ros_interface/GetCloudSmoothMode.h>
#include <mecheye_ros_interface/GetCurrentUserSet.h>
#include <mecheye_ros_interface/GetDepthRange.h>
#include <mecheye_ros_interface/GetFringeContrastThreshold.h>
#include <mecheye_ros_interface/GetFringeMinThreshold.h>
#include <mecheye_ros_interface/GetLaserSettings.h>
#include <mecheye_ros_interface/SaveAllSettingsToUserSets.h>
#include <mecheye_ros_interface/Set2DExpectedGrayValue.h>
#include <mecheye_ros_interface/Set2DExposureMode.h>
#include <mecheye_ros_interface/Set2DExposureSequence.h>
#include <mecheye_ros_interface/Set2DExposureTime.h>
#include <mecheye_ros_interface/Set2DROI.h>
#include <mecheye_ros_interface/Set2DSharpenFactor.h>
#include <mecheye_ros_interface/Set2DToneMappingEnable.h>
#include <mecheye_ros_interface/Set3DExposure.h>
#include <mecheye_ros_interface/Set3DGain.h>
#include <mecheye_ros_interface/Set3DROI.h>
#include <mecheye_ros_interface/SetCloudOutlierFilterMode.h>
#include <mecheye_ros_interface/SetCloudSmoothMode.h>
#include <mecheye_ros_interface/SetCurrentUserSet.h>
#include <mecheye_ros_interface/SetDepthRange.h>
#include <mecheye_ros_interface/SetFringeContrastThreshold.h>
#include <mecheye_ros_interface/SetFringeMinThreshold.h>
#include <mecheye_ros_interface/SetLaserSettings.h>

namespace mecheye_ros_interface
{

class MechMindCamera
{
public:
    MechMindCamera();

private:
    mmind::api::MechEyeDevice device;
    mmind::api::DeviceIntri intri;

    ros::NodeHandle nh;

    std::string camera_ip;
    bool save_file = false;
    bool use_external_intri = false;
    double fx = 0;
    double fy = 0;
    double u = 0;
    double v = 0;

    ros::Publisher pub_color;
    ros::Publisher pub_depth;
    ros::Publisher pub_pcl;
    ros::Publisher pub_pcl_color;
    ros::Publisher pub_camera_info;

    void publishColorMap(mmind::api::ColorMap& colorMap);
    void publishDepthMap(mmind::api::DepthMap& depthMap);
    void publishPointCloud(mmind::api::PointXYZMap& pointXYZMap);
    void publishColorPointCloud(mmind::api::PointXYZBGRMap& pointXYZBGRMap);
    void publishCameraInfo(const std_msgs::Header& header, int width, int height);

    ros::ServiceServer add_user_set_service;
    ros::ServiceServer capture_color_map_service;
    ros::ServiceServer capture_color_point_cloud_service;
    ros::ServiceServer capture_depth_map_service;
    ros::ServiceServer capture_point_cloud_service;
    ros::ServiceServer delete_user_set_service;
    ros::ServiceServer device_info_service;
    ros::ServiceServer get_2d_expected_gray_value_service;
    ros::ServiceServer get_2d_exposure_mode_service;
    ros::ServiceServer get_2d_exposure_sequence_service;
    ros::ServiceServer get_2d_exposure_time_service;
    ros::ServiceServer get_2d_roi_service;
    ros::ServiceServer get_2d_sharpen_factor_service;
    ros::ServiceServer get_2d_tone_mapping_service;
    ros::ServiceServer get_3d_exposure_service;
    ros::ServiceServer get_3d_gain_service;
    ros::ServiceServer get_3d_roi_service;
    ros::ServiceServer get_all_user_sets_service;
    ros::ServiceServer get_cloud_outlier_filter_mode_service;
    ros::ServiceServer get_cloud_smooth_mode_service;
    ros::ServiceServer get_current_user_set_service;
    ros::ServiceServer get_depth_range_service;
    ros::ServiceServer get_fringe_contrast_threshold_service;
    ros::ServiceServer get_fringe_min_threshold_service;
    ros::ServiceServer get_laser_settings_service;
    ros::ServiceServer save_all_settings_to_user_sets_service;
    ros::ServiceServer set_2d_expected_gray_value_service;
    ros::ServiceServer set_2d_exposure_mode_service;
    ros::ServiceServer set_2d_exposure_sequence_service;
    ros::ServiceServer set_2d_exposure_time_service;
    ros::ServiceServer set_2d_roi_service;
    ros::ServiceServer set_2d_sharpen_factor_service;
    ros::ServiceServer set_2d_tone_mapping_service;
    ros::ServiceServer set_3d_exposure_service;
    ros::ServiceServer set_3d_gain_service;
    ros::ServiceServer set_3d_roi_service;
    ros::ServiceServer set_cloud_outlier_filter_mode_service;
    ros::ServiceServer set_cloud_smooth_mode_service;
    ros::ServiceServer set_current_user_set_service;
    ros::ServiceServer set_depth_range_service;
    ros::ServiceServer set_fringe_contrast_threshold_service;
    ros::ServiceServer set_fringe_min_threshold_service;
    ros::ServiceServer set_laser_settings_service;

    bool add_user_set_callback(AddUserSet::Request& req, AddUserSet::Response& res);
    bool capture_color_map_callback(CaptureColorMap::Request& req, CaptureColorMap::Response& res);
    bool capture_color_point_cloud_callback(CaptureColorPointCloud::Request& req, CaptureColorPointCloud::Response& res);
    bool capture_depth_map_callback(CaptureDepthMap::Request& req, CaptureDepthMap::Response& res);
    bool capture_point_cloud_callback(CapturePointCloud::Request& req, CapturePointCloud::Response& res);
    bool delete_user_set_callback(DeleteUserSet::Request& req, DeleteUserSet::Response& res);
    bool device_info_callback(DeviceInfo::Request& req, DeviceInfo::Response& res);
    bool get_2d_expected_gray_value_callback(Get2DExpectedGrayValue::Request& req,
                                             Get2DExpectedGrayValue::Response& res);
    bool get_2d_exposure_mode_callback(Get2DExposureMode::Request& req, Get2DExposureMode::Response& res);
    bool get_2d_exposure_sequence_callback(Get2DExposureSequence::Request& req, Get2DExposureSequence::Response& res);
    bool get_2d_exposure_time_callback(Get2DExposureTime::Request& req, Get2DExposureTime::Response& res);
    bool get_2d_roi_callback(Get2DROI::Request& req, Get2DROI::Response& res);
    bool get_2d_sharpen_factor_callback(Get2DSharpenFactor::Request& req, Get2DSharpenFactor::Response& res);
    bool get_2d_tone_mapping_callback(Get2DToneMappingEnable::Request& req, Get2DToneMappingEnable::Response& res);
    bool get_3d_exposure_callback(Get3DExposure::Request& req, Get3DExposure::Response& res);
    bool get_3d_gain_callback(Get3DGain::Request& req, Get3DGain::Response& res);
    bool get_3d_roi_callback(Get3DROI::Request& req, Get3DROI::Response& res);
    bool get_all_user_sets_callback(GetAllUserSets::Request& req, GetAllUserSets::Response& res);
    bool get_cloud_outlier_filter_mode_callback(GetCloudOutlierFilterMode::Request& req,
                                                GetCloudOutlierFilterMode ::Response& res);
    bool get_cloud_smooth_mode_callback(GetCloudSmoothMode::Request& req, GetCloudSmoothMode::Response& res);
    bool get_current_user_set_callback(GetCurrentUserSet::Request& req, GetCurrentUserSet::Response& res);
    bool get_depth_range_callback(GetDepthRange::Request& req, GetDepthRange::Response& res);
    bool get_fringe_contrast_threshold_callback(GetFringeContrastThreshold::Request& req,
                                                GetFringeContrastThreshold::Response& res);
    bool get_fringe_min_threshold_callback(GetFringeMinThreshold::Request& req, GetFringeMinThreshold::Response& res);
    bool get_laser_settings_callback(GetLaserSettings::Request& req, GetLaserSettings::Response& res);
    bool save_all_settings_to_user_sets_callback(SaveAllSettingsToUserSets::Request& req,
                                                 SaveAllSettingsToUserSets::Response& res);
    bool set_2d_expected_gray_value_callback(Set2DExpectedGrayValue::Request& req,
                                             Set2DExpectedGrayValue::Response& res);
    bool set_2d_exposure_mode_callback(Set2DExposureMode::Request& req, Set2DExposureMode::Response& res);
    bool set_2d_exposure_sequence_callback(Set2DExposureSequence::Request& req, Set2DExposureSequence::Response& res);
    bool set_2d_exposure_time_callback(Set2DExposureTime::Request& req, Set2DExposureTime::Response& res);
    bool set_2d_roi_callback(Set2DROI::Request& req, Set2DROI::Response& res);
    bool set_2d_sharpen_factor_callback(Set2DSharpenFactor::Request& req, Set2DSharpenFactor::Response& res);
    bool set_2d_tone_mapping_callback(Set2DToneMappingEnable::Request& req, Set2DToneMappingEnable::Response& res);
    bool set_3d_exposure_callback(Set3DExposure::Request& req, Set3DExposure::Response& res);
    bool set_3d_gain_callback(Set3DGain::Request& req, Set3DGain::Response& res);
    bool set_3d_roi_callback(Set3DROI::Request& req, Set3DROI::Response& res);
    bool set_cloud_outlier_filter_mode_callback(SetCloudOutlierFilterMode::Request& req,
                                                SetCloudOutlierFilterMode ::Response& res);
    bool set_cloud_smooth_mode_callback(SetCloudSmoothMode::Request& req, SetCloudSmoothMode::Response& res);
    bool set_current_user_set_callback(SetCurrentUserSet::Request& req, SetCurrentUserSet::Response& res);
    bool set_depth_range_callback(SetDepthRange::Request& req, SetDepthRange::Response& res);
    bool set_fringe_contrast_threshold_callback(SetFringeContrastThreshold::Request& req,
                                                SetFringeContrastThreshold::Response& res);
    bool set_fringe_min_threshold_callback(SetFringeMinThreshold::Request& req, SetFringeMinThreshold::Response& res);
    bool set_laser_settings_callback(SetLaserSettings::Request& req, SetLaserSettings::Response& res);
};

}  // namespace mecheye_ros_interface