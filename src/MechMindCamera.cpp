#include "MechMindCamera.h"
#include <SampleUtil.h>
#include <OpenCVUtil.h>
#include <PclUtil.h>
#include <opencv2/imgcodecs.hpp>
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

namespace mecheye_ros_interface
{

MechMindCamera::MechMindCamera()
{
    nh.getParam("camera_ip", camera_ip);
    nh.getParam("save_file", save_file);
    nh.getParam("use_external_intri", use_external_intri);
    nh.getParam("fx", fx);
    nh.getParam("fy", fy);
    nh.getParam("u", u);
    nh.getParam("v", v);

    pub_color = nh.advertise<sensor_msgs::Image>("/mechmind/color_image", 1, true);
    pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind/depth_image", 1, true);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/point_cloud", 1, true);
    pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/color_point_cloud", 1, true);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/mechmind/camera_info", 1, true);

    if (!findAndConnect(device))
        return;

    // Uncomment the following lines to connect a camera with ip inside .launch file

    // mmind::api::ErrorStatus status;
    // mmind::api::MechEyeDeviceInfo info;
    // info.firmwareVersion = "1.5.0";
    // info.ipAddress = camera_ip;
    // info.port = 5577;
    // status = device.connect(info);
    // if (!status.isOK())
    // {
    //     showError(status);
    //     return;
    // }
    // std::cout << "Connected to the Mech-Eye device successfully." << std::endl;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
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

    add_user_set_service = nh.advertiseService("add_user_set", &MechMindCamera::add_user_set_callback, this);
    capture_color_map_service =
        nh.advertiseService("capture_color_map", &MechMindCamera::capture_color_map_callback, this);
    capture_color_point_cloud_service = nh.advertiseService("capture_color_point_cloud_service",
                                                            &MechMindCamera::capture_color_point_cloud_callback, this);
    capture_depth_map_service =
        nh.advertiseService("capture_depth_map", &MechMindCamera::capture_depth_map_callback, this);
    capture_point_cloud_service =
        nh.advertiseService("capture_point_cloud", &MechMindCamera::capture_point_cloud_callback, this);
    delete_user_set_service = nh.advertiseService("delete_user_set", &MechMindCamera::delete_user_set_callback, this);
    get_2d_expected_gray_value_service =
        nh.advertiseService("get_2d_expected_gray_value", &MechMindCamera::get_2d_expected_gray_value_callback, this);
    get_2d_exposure_mode_service =
        nh.advertiseService("get_2d_exposure_mode", &MechMindCamera::get_2d_exposure_mode_callback, this);
    get_2d_exposure_sequence_service =
        nh.advertiseService("get_2d_exposure_sequence", &MechMindCamera::get_2d_exposure_sequence_callback, this);
    get_2d_exposure_time_service =
        nh.advertiseService("get_2d_exposure_time", &MechMindCamera::get_2d_exposure_time_callback, this);
    get_2d_roi_service = nh.advertiseService("get_2d_roi", &MechMindCamera::get_2d_roi_callback, this);
    get_2d_sharpen_factor_service =
        nh.advertiseService("get_2d_sharpen_factor", &MechMindCamera::get_2d_sharpen_factor_callback, this);
    get_2d_tone_mapping_service =
        nh.advertiseService("get_2d_tone_mapping", &MechMindCamera::get_2d_tone_mapping_callback, this);
    get_3d_exposure_service = nh.advertiseService("get_3d_exposure", &MechMindCamera::get_3d_exposure_callback, this);
    get_3d_gain_service = nh.advertiseService("get_3d_gain", &MechMindCamera::get_3d_gain_callback, this);
    get_3d_roi_service = nh.advertiseService("get_3d_roi", &MechMindCamera::get_3d_roi_callback, this);
    get_all_user_sets_service =
        nh.advertiseService("get_all_user_sets", &MechMindCamera::get_all_user_sets_callback, this);
    get_cloud_outlier_filter_mode_service = nh.advertiseService(
        "get_cloud_outlier_filter_mode", &MechMindCamera::get_cloud_outlier_filter_mode_callback, this);
    get_cloud_smooth_mode_service =
        nh.advertiseService("get_cloud_smooth_mode", &MechMindCamera::get_cloud_smooth_mode_callback, this);
    get_current_user_set_service =
        nh.advertiseService("get_current_user_set", &MechMindCamera::get_current_user_set_callback, this);
    get_depth_range_service = nh.advertiseService("get_depth_range", &MechMindCamera::get_depth_range_callback, this);
    get_fringe_contrast_threshold_service = nh.advertiseService(
        "get_fringe_contrast_threshold", &MechMindCamera::get_fringe_contrast_threshold_callback, this);
    get_fringe_min_threshold_service =
        nh.advertiseService("get_fringe_min_threshold", &MechMindCamera::get_fringe_min_threshold_callback, this);
    get_laser_settings_service =
        nh.advertiseService("get_laser_settings", &MechMindCamera::get_laser_settings_callback, this);
    set_2d_expected_gray_value_service =
        nh.advertiseService("set_2d_expected_gray_value", &MechMindCamera::set_2d_expected_gray_value_callback, this);
    set_2d_exposure_mode_service =
        nh.advertiseService("set_2d_exposure_mode", &MechMindCamera::set_2d_exposure_mode_callback, this);
    set_2d_exposure_sequence_service =
        nh.advertiseService("set_2d_exposure_sequence", &MechMindCamera::set_2d_exposure_sequence_callback, this);
    set_2d_exposure_time_service =
        nh.advertiseService("set_2d_exposure_time", &MechMindCamera::set_2d_exposure_time_callback, this);
    set_2d_roi_service = nh.advertiseService("set_2d_roi", &MechMindCamera::set_2d_roi_callback, this);
    set_2d_sharpen_factor_service =
        nh.advertiseService("set_2d_sharpen_factor", &MechMindCamera::set_2d_sharpen_factor_callback, this);
    set_2d_tone_mapping_service =
        nh.advertiseService("set_2d_tone_mapping", &MechMindCamera::set_2d_tone_mapping_callback, this);
    set_3d_exposure_service = nh.advertiseService("set_3d_exposure", &MechMindCamera::set_3d_exposure_callback, this);
    set_3d_gain_service = nh.advertiseService("set_3d_gain", &MechMindCamera::set_3d_gain_callback, this);
    set_3d_roi_service = nh.advertiseService("set_3d_roi", &MechMindCamera::set_3d_roi_callback, this);
    set_cloud_outlier_filter_mode_service = nh.advertiseService(
        "set_cloud_outlier_filter_mode", &MechMindCamera::set_cloud_outlier_filter_mode_callback, this);
    set_cloud_smooth_mode_service =
        nh.advertiseService("set_cloud_smooth_mode", &MechMindCamera::set_cloud_smooth_mode_callback, this);
    set_current_user_set_service =
        nh.advertiseService("set_current_user_set", &MechMindCamera::set_current_user_set_callback, this);
    set_depth_range_service = nh.advertiseService("set_depth_range", &MechMindCamera::set_depth_range_callback, this);
    set_fringe_contrast_threshold_service = nh.advertiseService(
        "set_fringe_contrast_threshold", &MechMindCamera::set_fringe_contrast_threshold_callback, this);
    set_fringe_min_threshold_service =
        nh.advertiseService("set_fringe_min_threshold", &MechMindCamera::set_fringe_min_threshold_callback, this);
    set_laser_settings_service =
        nh.advertiseService("set_laser_settings", &MechMindCamera::set_laser_settings_callback, this);
}

void MechMindCamera::publishColorMap(mmind::api::ColorMap& colorMap)
{
    cv::Mat color = cv::Mat(colorMap.height(), colorMap.width(), CV_8UC3, colorMap.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_camera";
    ros_image.header.stamp = ros::Time::now();
    pub_color.publish(ros_image);
    if (save_file)
        saveMap(colorMap, "/tmp/mechmind_color.png");
}

void MechMindCamera::publishDepthMap(mmind::api::DepthMap& depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_camera";
    ros_depth.header.stamp = ros::Time::now();
    pub_depth.publish(ros_depth);
    if (save_file)
        saveMap(depthMap, "/tmp/mechmind_depth.png")
}

void MechMindCamera::publishPointCloud(mmind::api::PointXYZMap& pointXYZMap)
{
    pcl::PointCloud<pcl::PointXYZ> cloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(cloud, pointXYZMap);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.frame_id = "mechmind_camera";
    ros_cloud.header.stamp = ros::Time::now();
    pub_pcl.publish(ros_cloud);
    if (save_file)
        savePLY(pointXYZMap, "/tmp/mechmind_cloud.ply")
}

void MechMindCamera::publishColorPointCloud(mmind::api::PointXYZBGRMap& pointXYZBGRMap)
{
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(color_cloud, pointXYZBGRMap);
    sensor_msgs::PointCloud2 ros_color_cloud;
    pcl::toROSMsg(color_cloud, ros_color_cloud);
    ros_color_cloud.header.frame_id = "mechmind_camera";
    ros_color_cloud.header.stamp = ros::Time::now();
    pub_pcl_color.publish(ros_color_cloud);
    if (save_file)
        savePLY(pointXYZBGRMap, "/tmp/mechmind_color_cloud.ply")
}

bool MechMindCamera::add_user_set_callback(AddUserSet::Request& req, AddUserSet::Response& res)
{
    mmind::api::ErrorStatus status = device.addUserSet({ req.value.c_str() });
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::capture_color_map_callback(CaptureColorMap::Request& req, CaptureColorMap::Response& res)
{
    mmind::api::ColorMap colorMap;
    mmind::api::ErrorStatus status = device.captureColorMap(colorMap);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    publishColorMap(colorMap);
    return status.isOK();
}

bool MechMindCamera::capture_color_point_cloud_callback(CaptureColorPointCloud::Request& req,
                                                        CaptureColorPointCloud::Response& res)
{
    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    mmind::api::ErrorStatus status = device.capturePointXYZBGRMap(pointXYZBGRMap);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    publishColorPointCloud(pointXYZBGRMap);
    return status.isOK();
}

bool MechMindCamera::capture_depth_map_callback(CaptureDepthMap::Request& req, CaptureDepthMap::Response& res)
{
    mmind::api::DepthMap depthMap;
    mmind::api::ErrorStatus status = device.captureDepthMap(depthMap);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    publishDepthMap(depthMap);
    return status.isOK();
}

bool MechMindCamera::capture_point_cloud_callback(CapturePointCloud::Request& req, CapturePointCloud::Response& res)
{
    mmind::api::PointXYZMap pointXYZMap;
    mmind::api::ErrorStatus status = device.capturePointXYZMap(pointXYZMap);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    publishPointCloud(pointXYZMap);
    return status.isOK();
}

bool MechMindCamera::delete_user_set_callback(DeleteUserSet::Request& req, DeleteUserSet::Response& res)
{
    mmind::api::ErrorStatus status = device.deleteUserSet({ req.value.c_str() });
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::device_info_callback(DeviceInfo::Request& req, DeviceInfo::Response& res)
{
    mmind::api::MechEyeDeviceInfo deviceInfo;
    mmind::api::ErrorStatus status = device.getDeviceInfo(deviceInfo).isOK();
    res.model = deviceInfo.model.c_str();
    res.id = deviceInfo.id.c_str();
    res.hardwareVersion = deviceInfo.hardwareVersion.c_str();
    res.firmwareVersion = deviceInfo.firmwareVersion.c_str();
    res.ipAddress = deviceInfo.ipAddress.c_str();
    res.port = deviceInfo.port;
    return status.isOK();
}

bool MechMindCamera::get_2d_expected_gray_value_callback(Get2DExpectedGrayValue::Request& req,
                                                         Get2DExpectedGrayValue::Response& res)
{
    int value;
    mmind::api::ErrorStatus status = device.getScan2DExpectedGrayValue(value);
    res.value = value;
    return status.isOK();
}
bool MechMindCamera::get_2d_exposure_mode_callback(Get2DExposureMode::Request& req, Get2DExposureMode::Response& res)
{
    int mode = 4;
    mmind::api::ErrorStatus status = device.getScan2DExposureMode(mode);
    switch (mode)
    {
        case 0:
            res.value = "Timed";
            break;

        case 1:
            res.value = "Auto";
            break;

        case 2:
            res.value = "HDR";
            break;

        case 3:
            res.value = "Flash";
            break;

        default:
            res.value = "";
            break;
    }
    return status.isOK();
}

bool MechMindCamera::get_2d_exposure_sequence_callback(Get2DExposureSequence::Request& req,
                                                       Get2DExposureSequence::Response& res)
{
    std::vector<double> sequence;
    mmind::api::ErrorStatus status = device.getScan2DHDRExposureSequence(sequence);
    res.sequence = sequence;
    return status.isOK();
}

bool MechMindCamera::get_2d_exposure_time_callback(Get2DExposureTime::Request& req, Get2DExposureTime::Response& res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan2DExposureTime(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_2d_roi_callback(Get2DROI::Request& req, Get2DROI::Response& res)
{
    mmind::api::ROI roi;
    mmind::api::ErrorStatus status = device.getScan2DROI(roi);
    res.x = roi.x;
    res.y = roi.y;
    res.width = roi.width;
    res.height = roi.height;
    return status.isOK();
}

bool MechMindCamera::get_2d_sharpen_factor_callback(Get2DSharpenFactor::Request& req, Get2DSharpenFactor::Response& res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan2DSharpenFactor(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_2d_tone_mapping_callback(Get2DToneMappingEnable::Request& req,
                                                  Get2DToneMappingEnable::Response& res)
{
    bool value;
    mmind::api::ErrorStatus status = devie.getScan2DToneMappingEnable(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_3d_exposure_callback(Get3DExposure::Request& req, Get3DExposure::Response& res)
{
    std::vector<double> sequence;
    mmind::api::ErrorStatus status = devie.getScan3DExposure(sequence);
    res.sequence = sequence;
    return status.isOK();
}

bool MechMindCamera::get_3d_gain_callback(Get3DGain::Request& req, Get3DGain::Response& res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan3DGain(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_3d_roi_callback(Get3DROI::Request& req, Get3DROI::Response& res)
{
    mmind::api::ROI roi;
    mmind::api::ErrorStatus status = device.getScan3DROI(roi);
    res.x = roi.x;
    res.y = roi.y;
    res.width = roi.width;
    res.height = roi.height;
    return status.isOK();
}

bool MechMindCamera::get_all_user_sets_callback(GetAllUserSets::Request& req, GetAllUserSets::Response& res)
{
    std::vector<std::string> sequence;
    mmind::api::ErrorStatus status = device.getAllUserSets(sequence);
    std::vector<char*> charPtrSequence;
    for (auto& s : sequence)
    {
        charPtrSequence.emplace_back(s.c_str());
    }
    res.sequence = charPtrSequence;
    return status.isOK();
}

bool MechMindCamera::get_cloud_outlier_filter_mode_callback(GetCloudOutlierFilterMode::Request& req,
                                                            GetCloudOutlierFilterMode ::Response& res)
{
    int mode = 3;
    mmind::api::ErrorStatus status = device.getCloudOutlierFilterMode(mode);
    switch (mode)
    {
        case 0:
            res.value = "Off";
            break;

        case 1:
            res.value = "Normal";
            break;

        case 2:
            res.value = "Weak";
            break;

        default:
            res.value = "";
            break;
    }
    return status.isOK();
}

bool MechMindCamera::get_cloud_smooth_mode_callback(GetCloudSmoothMode::Request& req, GetCloudSmoothMode::Response& res)
{
    int mode = 4;
    mmind::api::ErrorStatus status = device.getCloudSmoothMode(mode);
    switch (mode)
    {
        case 0:
            res.value = "Off";
            break;

        case 1:
            res.value = "Normal";
            break;

        case 2:
            res.value = "Weak";
            break;

        case 3:
            res.value = "Strong";
            break;

        default:
            res.value = "";
            break;
    }
    return status.isOK();
}

bool MechMindCamera::get_current_user_set_callback(GetCurrentUserSet::Request& req, GetCurrentUserSet::Response& res)
{
    std::string value;
    mmind::api::ErrorStatus status = device.getCurrentUserSet(value);
    res.value = value.c_str();
    return status.isOK();
}

bool MechMindCamera::get_depth_range_callback(GetDepthRange::Request& req, GetDepthRange::Response& res)
{
    mmind::api::DepthRange depthRange;
    mmind::api::ErrorStatus status = device.getDepthRange(depthRange);
    res.lower = depthRange.lower;
    res.upper = depthRange.upper;
    return status.isOK();
}

bool MechMindCamera::get_fringe_contrast_threshold_callback(GetFringeContrastThreshold::Request& req,
                                                            GetFringeContrastThreshold::Response& res)
{
    int value;
    mmind::api::ErrorStatus status = device.getFringeContrastThreshold(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_fringe_min_threshold_callback(GetFringeMinThreshold::Request& req,
                                                       GetFringeMinThreshold::Response& res)
{
    int value;
    mmind::api::ErrorStatus status = device.getFringeMinThreshold(value);
    res.value = value;
    return status.isOK();
}

bool MechMindCamera::get_laser_settings(GetLaserSettings::Request& req, GetLaserSettings::Response& res)
{
    mmind::api::LaserSettings laserSettings{ 2, -1, -1, -1, -1 };
    mmind::api::ErrorStatus status = device.getLaserSettings(laserSettings);
    switch (laserSettings.FringeCodingMode)
    {
        case 0:
            res.fringeCodingMode = "Fast";
            break;

        case 1:
            res.fringeCodingMode = "High";
            break;

        default:
            res.fringeCodingMode = "";
            break;
    }
    res.frameRangeStart = laserSettings.FrameRangeStart;
    res.frameRangeEnd = laserSettings.FrameRangeEnd;
    res.framePartitionCount = laserSettings.FramePartitionCount;
    res.powerLevel = laserSettings.PowerLevel;
    return status.isOK();
}

bool MechMindCamera::save_all_settings_to_user_sets_callback(SaveAllSettingsToUserSets::Request& req,
                                                             SaveAllSettingsToUserSets::Response& res)
{
    return device.saveAllSettingsToUserSets().isOK();
}

bool MechMindCamera::set_2d_expected_gray_value_callback(Set2DExpectedGrayValue::Request& req,
                                                         Set2DExpectedGrayValue::Response& res)
{
    mmind::api::ErrorStatus status = device.setScan2DExpectedGrayValue(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_exposure_mode_callback(Set2DExposureMode::Request& req, Set2DExposureMode::Response& res)
{
    int mode;
    switch (req.value.c_str())
    {
        case "Timed":
            mode = 0;
            break;

        case "Auto":
            mode = 1;
            break;

        case "HDR":
            mode = 2;
            break;

        case "Flash":
            mode = 3;
            break;

        default:
            return false;
    }
    mmind::api::ErrorStatus status = device.setScan2DExposureMode(mode);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_exposure_sequence_callback(Set2DExposureSequence::Request& req,
                                                       Set2DExposureSequence::Response& res)
{
    std::vector<double> sequence(begin(req.sequence), end(req.sequence));
    mmind::api::ErrorStatus status = device.setScan2DHDRExposureSequence(sequence);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_exposure_time_callback(Set2DExposureTime::Request& req, Set2DExposureTime::Response& res)
{
    mmind::api::ErrorStatus status = device.setScan2DExposureTime(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_roi_callback(Set2DROI::Request& req, Set2DROI::Response& res)
{
    mmind::api::ROI roi{ req.x, req.y, req.width, req.height };
    mmind::api::ErrorStatus status = device.setScan2DROI(roi);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_sharpen_factor_callback(Set2DSharpenFactor::Request& req, Set2DSharpenFactor::Response& res)
{
    mmind::api::ErrorStatus status = device.setScan2DSharpenFactor(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_2d_tone_mapping_callback(Set2DToneMappingEnable::Request& req,
                                                  Set2DToneMappingEnable::Response& res)
{
    mmind::api::ErrorStatus status = device.setScan2DToneMappingEnable(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_3d_exposure_callback(Set3DExposure::Request& req, Set3DExposure::Response& res)
{
    std::vector<double> sequence(begin(req.sequence), end(req.sequence));
    mmind::api::ErrorStatus status = device.setScan3DExposure(sequence);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_3d_gain_callback(Set3DGain::Request& req, Set3DGain::Response& res)
{
    mmind::api::ErrorStatus status = device.setScan3DGain(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_3d_roi_callback(Set3DROI::Request& req, Set3DROI::Response& res)
{
    mmind::api::ROI roi{ req.x, req.y, req.width, req.height };
    mmind::api::ErrorStatus status = device.setScan3DROI(roi);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_cloud_outlier_filter_mode_callback(SetCloudOutlierFilterMode::Request& req,
                                                            SetCloudOutlierFilterMode ::Response& res)
{
    int mode;
    switch (req.value.c_str())
    {
        case "Off":
            mode = 0;
            break;

        case "Normal":
            mode = 1;
            break;

        case "Weak":
            mode = 2;
            break;

        default:
            return false;
    }
    mmind::api::ErrorStatus status = device.setCloudOutlierFilterMode(mode);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_cloud_smooth_mode_callback(SetCloudSmoothMode::Request& req, SetCloudSmoothMode::Response& res)
{
    int mode;
    switch (req.value.c_str())
    {
        case "Off":
            mode = 0;
            break;

        case "Normal":
            mode = 1;
            break;

        case "Weak":
            mode = 2;
            break;

        case "Strong":
            mode = 3;
            break;

        default:
            return false;
    }
    mmind::api::ErrorStatus status = device.setCloudSmoothMode(mode);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_current_user_set_callback(SetCurrentUserSet::Request& req, SetCurrentUserSet::Response& res)
{
    mmind::api::ErrorStatus status = device.setCurrentUserSet({ req.value.c_str() });
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_depth_range_callback(SetDepthRange::Request& req, SetDepthRange::Response& res)
{
    mmind::api::DepthRange depthRange{ req.lower, req.upper };
    mmind::api::ErrorStatus status = device.setDepthRange(depthRange);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_fringe_contrast_threshold_callback(SetFringeContrastThreshold::Request& req,
                                                            SetFringeContrastThreshold::Response& res)
{
    mmind::api::ErrorStatus status = device.setFringeContrastThreshold(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_fringe_min_threshold_callback(SetFringeMinThreshold::Request& req,
                                                       SetFringeMinThreshold::Response& res)
{
    mmind::api::ErrorStatus status = device.setFringeMinThreshold(req.value);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

bool MechMindCamera::set_laser_settings(SetLaserSettings::Request& req, SetLaserSettings::Response& res)
{
    mmind::api::LaserSettings laserSettings{req.fringeCodingMode == "Fast" ? 0 : 1, req.frameRangeStart, req.frameRangeEnd, req.framePartitionCount, req.powerLevel};
    mmind::api::ErrorStatus status = device.setLaserSettings(laserSettings);
    res.errorCode = status.errorCode;
    res.errorDescription = status.errorDescription.c_str();
    return status.isOK();
}

}  // namespace mecheye_ros_interface