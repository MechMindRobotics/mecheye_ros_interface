#pragma once
#include <profiler/Profiler.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mecheye_profiler_ros_interface/srv/start_acquisition.hpp>
#include <mecheye_profiler_ros_interface/srv/stop_acquisition.hpp>
#include <mecheye_profiler_ros_interface/srv/trigger_software.hpp>
#include <mecheye_profiler_ros_interface/srv/delete_user_set.hpp>
#include <mecheye_profiler_ros_interface/srv/profiler_info.hpp>
#include <mecheye_profiler_ros_interface/srv/add_user_set.hpp>
#include <mecheye_profiler_ros_interface/srv/get_all_user_sets.hpp>
#include <mecheye_profiler_ros_interface/srv/get_current_user_set.hpp>
#include <mecheye_profiler_ros_interface/srv/save_all_settings_to_user_sets.hpp>
#include <mecheye_profiler_ros_interface/srv/set_current_user_set.hpp>
#include <mecheye_profiler_ros_interface/srv/set_int_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/get_int_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/set_bool_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/get_bool_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/set_float_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/get_float_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/set_enum_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/get_enum_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/set_profile_roi_parameter.hpp>
#include <mecheye_profiler_ros_interface/srv/get_profile_roi_parameter.hpp>

class MechMindProfiler
{
public:
    MechMindProfiler();
    void handleCallbackBatch(const mmind::eye::ProfileBatch& batch);
    rclcpp::Node::SharedPtr node;

private:
    mmind::eye::Profiler profiler;

    std::string profiler_ip;
    bool save_file = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_intensity;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_textured_pcl;

    void publishIntensityImage(mmind::eye::ProfileBatch::IntensityImage&& intensityImage);
    void publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap);
    void publishPointClouds(const mmind::eye::ProfileBatch& batch,
                            const mmind::eye::UserSet& userSet);

    rclcpp::Service<mecheye_profiler_ros_interface::srv::StartAcquisition>::SharedPtr
        start_acquisition_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::StopAcquisition>::SharedPtr
        stop_acquisition_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::TriggerSoftware>::SharedPtr
        trigger_software_service;

    rclcpp::Service<mecheye_profiler_ros_interface::srv::AddUserSet>::SharedPtr
        add_user_set_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::DeleteUserSet>::SharedPtr
        delete_user_set_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::ProfilerInfo>::SharedPtr
        profiler_info_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetAllUserSets>::SharedPtr
        get_all_user_sets_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetCurrentUserSet>::SharedPtr
        get_current_user_set_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets>::SharedPtr
        save_all_settings_to_user_sets_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetCurrentUserSet>::SharedPtr
        set_current_user_set_service;

    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetProfileROIParameter>::SharedPtr
        get_profile_roi_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetProfileROIParameter>::SharedPtr
        set_profile_roi_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetIntParameter>::SharedPtr
        get_int_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetIntParameter>::SharedPtr
        set_int_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetBoolParameter>::SharedPtr
        get_bool_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetBoolParameter>::SharedPtr
        set_bool_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetEnumParameter>::SharedPtr
        get_enum_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetEnumParameter>::SharedPtr
        set_enum_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::GetFloatParameter>::SharedPtr
        get_float_parameter_service;
    rclcpp::Service<mecheye_profiler_ros_interface::srv::SetFloatParameter>::SharedPtr
        set_float_parameter_service;

    void start_acquisition_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::StartAcquisition::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::StartAcquisition::Response> res);

    void stop_acquisition_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::StopAcquisition::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::StopAcquisition::Response> res);

    void trigger_software_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::TriggerSoftware::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::TriggerSoftware::Response> res);

    void add_user_set_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::AddUserSet::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::AddUserSet::Response> res);

    void delete_user_set_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::DeleteUserSet::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::DeleteUserSet::Response> res);

    void profiler_info_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::ProfilerInfo::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::ProfilerInfo::Response> res);

    void get_all_user_sets_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetAllUserSets::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetAllUserSets::Response> res);

    void get_current_user_set_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetCurrentUserSet::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetCurrentUserSet::Response> res);

    void save_all_settings_to_user_sets_callback(
        const std::shared_ptr<
            mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets::Request>
            req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets::Response>
            res);

    void set_current_user_set_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetCurrentUserSet::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetCurrentUserSet::Response> res);

    void get_profile_roi_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetProfileROIParameter::Request>
            req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetProfileROIParameter::Response> res);

    void set_profile_roi_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetProfileROIParameter::Request>
            req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetProfileROIParameter::Response> res);

    void get_int_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetIntParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetIntParameter::Response> res);

    void set_int_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetIntParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetIntParameter::Response> res);

    void get_bool_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetBoolParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetBoolParameter::Response> res);

    void set_bool_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetBoolParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetBoolParameter::Response> res);

    void get_enum_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetEnumParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetEnumParameter::Response> res);

    void set_enum_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetEnumParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetEnumParameter::Response> res);

    void get_float_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetFloatParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::GetFloatParameter::Response> res);

    void set_float_parameter_callback(
        const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetFloatParameter::Request> req,
        std::shared_ptr<mecheye_profiler_ros_interface::srv::SetFloatParameter::Response> res);
};
