#pragma once
#include <profiler/Profiler.h>
#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <mecheye_profiler_ros_interface/StartAcquisition.h>
#include <mecheye_profiler_ros_interface/StopAcquisition.h>
#include <mecheye_profiler_ros_interface/TriggerSoftware.h>
#include <mecheye_profiler_ros_interface/DeleteUserSet.h>
#include <mecheye_profiler_ros_interface/ProfilerInfo.h>
#include <mecheye_profiler_ros_interface/AddUserSet.h>
#include <mecheye_profiler_ros_interface/GetAllUserSets.h>
#include <mecheye_profiler_ros_interface/GetCurrentUserSet.h>
#include <mecheye_profiler_ros_interface/SaveAllSettingsToUserSets.h>
#include <mecheye_profiler_ros_interface/SetCurrentUserSet.h>
#include <mecheye_profiler_ros_interface/SetIntParameter.h>
#include <mecheye_profiler_ros_interface/GetIntParameter.h>
#include <mecheye_profiler_ros_interface/SetBoolParameter.h>
#include <mecheye_profiler_ros_interface/GetBoolParameter.h>
#include <mecheye_profiler_ros_interface/SetFloatParameter.h>
#include <mecheye_profiler_ros_interface/GetFloatParameter.h>
#include <mecheye_profiler_ros_interface/SetEnumParameter.h>
#include <mecheye_profiler_ros_interface/GetEnumParameter.h>
#include <mecheye_profiler_ros_interface/SetProfileROIParameter.h>
#include <mecheye_profiler_ros_interface/GetProfileROIParameter.h>

class MechMindProfiler
{
public:
    MechMindProfiler();
    void handleCallbackBatch(const mmind::eye::ProfileBatch& batch);

private:
    ros::NodeHandle nh;
    mmind::eye::Profiler profiler;

    std::string profiler_ip;
    bool save_file = false;

    ros::Publisher pub_intensity;
    ros::Publisher pub_depth;
    ros::Publisher pub_pcl;
    ros::Publisher pub_textured_pcl;

    void publishIntensityImage(mmind::eye::ProfileBatch::IntensityImage&& intensityImage);
    void publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap);
    void publishPointClouds(const mmind::eye::ProfileBatch& batch,
                            const mmind::eye::UserSet& userSet);

    ros::ServiceServer start_acquisition_service;
    ros::ServiceServer stop_acquisition_service;
    ros::ServiceServer trigger_software_service;

    ros::ServiceServer add_user_set_service;
    ros::ServiceServer delete_user_set_service;
    ros::ServiceServer profiler_info_service;
    ros::ServiceServer get_all_user_sets_service;
    ros::ServiceServer get_current_user_set_service;
    ros::ServiceServer save_all_settings_to_user_sets_service;
    ros::ServiceServer set_current_user_set_service;

    ros::ServiceServer get_profile_roi_parameter_service;
    ros::ServiceServer set_profile_roi_parameter_service;
    ros::ServiceServer get_int_parameter_service;
    ros::ServiceServer set_int_parameter_service;
    ros::ServiceServer get_bool_parameter_service;
    ros::ServiceServer set_bool_parameter_service;
    ros::ServiceServer get_enum_parameter_service;
    ros::ServiceServer set_enum_parameter_service;
    ros::ServiceServer get_float_parameter_service;
    ros::ServiceServer set_float_parameter_service;

    bool start_acquisition_callback(
        mecheye_profiler_ros_interface::StartAcquisition::Request& req,
        mecheye_profiler_ros_interface::StartAcquisition::Response& res);

    bool stop_acquisition_callback(mecheye_profiler_ros_interface::StopAcquisition::Request& req,
                                   mecheye_profiler_ros_interface::StopAcquisition::Response& res);
    bool trigger_software_callback(mecheye_profiler_ros_interface::TriggerSoftware::Request& req,
                                   mecheye_profiler_ros_interface::TriggerSoftware::Response& res);
    bool add_user_set_callback(mecheye_profiler_ros_interface::AddUserSet::Request& req,
                               mecheye_profiler_ros_interface::AddUserSet::Response& res);
    bool delete_user_set_callback(mecheye_profiler_ros_interface::DeleteUserSet::Request& req,
                                  mecheye_profiler_ros_interface::DeleteUserSet::Response& res);
    bool profiler_info_callback(mecheye_profiler_ros_interface::ProfilerInfo::Request& req,
                                mecheye_profiler_ros_interface::ProfilerInfo::Response& res);
    bool get_all_user_sets_callback(mecheye_profiler_ros_interface::GetAllUserSets::Request& req,
                                    mecheye_profiler_ros_interface::GetAllUserSets::Response& res);
    bool get_current_user_set_callback(
        mecheye_profiler_ros_interface::GetCurrentUserSet::Request& req,
        mecheye_profiler_ros_interface::GetCurrentUserSet::Response& res);
    bool save_all_settings_to_user_sets_callback(
        mecheye_profiler_ros_interface::SaveAllSettingsToUserSets::Request& req,
        mecheye_profiler_ros_interface::SaveAllSettingsToUserSets::Response& res);

    bool set_current_user_set_callback(
        mecheye_profiler_ros_interface::SetCurrentUserSet::Request& req,
        mecheye_profiler_ros_interface::SetCurrentUserSet::Response& res);

    bool get_profile_roi_parameter_callback(
        mecheye_profiler_ros_interface::GetProfileROIParameter::Request& req,
        mecheye_profiler_ros_interface::GetProfileROIParameter::Response& res);

    bool set_profile_roi_parameter_callback(
        mecheye_profiler_ros_interface::SetProfileROIParameter::Request& req,
        mecheye_profiler_ros_interface::SetProfileROIParameter::Response& res);

    bool get_int_parameter_callback(mecheye_profiler_ros_interface::GetIntParameter::Request& req,
                                    mecheye_profiler_ros_interface::GetIntParameter::Response& res);

    bool set_int_parameter_callback(mecheye_profiler_ros_interface::SetIntParameter::Request& req,
                                    mecheye_profiler_ros_interface::SetIntParameter::Response& res);

    bool get_bool_parameter_callback(
        mecheye_profiler_ros_interface::GetBoolParameter::Request& req,
        mecheye_profiler_ros_interface::GetBoolParameter::Response& res);

    bool set_bool_parameter_callback(
        mecheye_profiler_ros_interface::SetBoolParameter::Request& req,
        mecheye_profiler_ros_interface::SetBoolParameter::Response& res);

    bool get_enum_parameter_callback(
        mecheye_profiler_ros_interface::GetEnumParameter::Request& req,
        mecheye_profiler_ros_interface::GetEnumParameter::Response& res);

    bool set_enum_parameter_callback(
        mecheye_profiler_ros_interface::SetEnumParameter::Request& req,
        mecheye_profiler_ros_interface::SetEnumParameter::Response& res);

    bool get_float_parameter_callback(
        mecheye_profiler_ros_interface::GetFloatParameter::Request& req,
        mecheye_profiler_ros_interface::GetFloatParameter::Response& res);

    bool set_float_parameter_callback(
        mecheye_profiler_ros_interface::SetFloatParameter::Request& req,
        mecheye_profiler_ros_interface::SetFloatParameter::Response& res);
};
