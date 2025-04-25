#include <MechMindProfiler.h>
#include <profiler/api_util.h>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/String.h>

namespace {

constexpr double kUmToM = 1e-6;
constexpr double kMmToM = 1e-3;

sensor_msgs::PointField createPointField(const std::string& name, uint32_t offset, uint8_t datatype,
                                         uint32_t count)
{
    sensor_msgs::PointField point_field;
    point_field.name = name;
    point_field.offset = offset;
    point_field.datatype = datatype;
    point_field.count = count;
    return point_field;
}

void convertToROSMsg(const mmind::eye::ProfileBatch& batch, float xResolution, float yResolution,
                     bool useEncoderValues, int triggerInterval, sensor_msgs::PointCloud2& cloud,
                     sensor_msgs::PointCloud2& texturedCloud)
{
    const auto height = batch.height();
    const auto width = batch.width();
    cloud.height = height;
    texturedCloud.height = height;
    cloud.width = width;
    texturedCloud.width = width;
    cloud.is_dense = false;
    texturedCloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    texturedCloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(float) * 3;
    texturedCloud.point_step = sizeof(float) * 4;
    cloud.row_step = cloud.point_step * cloud.width;
    texturedCloud.row_step = texturedCloud.point_step * texturedCloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
    texturedCloud.data.resize(texturedCloud.row_step * texturedCloud.height);

    cloud.fields.reserve(3);
    cloud.fields.push_back(createPointField("x", 0, sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(
        createPointField("y", sizeof(float), sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(
        createPointField("z", sizeof(float) * 2, sensor_msgs::PointField::FLOAT32, 1));

    texturedCloud.fields.reserve(4);
    texturedCloud.fields.push_back(createPointField("x", 0, sensor_msgs::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField("y", sizeof(float), sensor_msgs::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField("z", sizeof(float) * 2, sensor_msgs::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField("intensity", sizeof(float) * 3, sensor_msgs::PointField::UINT8, 1));

    memcpy(cloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(
               batch
                   .getUntexturedPointCloud(xResolution, yResolution, useEncoderValues,
                                            triggerInterval, mmind::eye::CoordinateUnit::Meter)
                   .data())),
           (cloud.row_step * cloud.height));
    memcpy(texturedCloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZI*>(
               batch
                   .getTexturedPointCloud(xResolution, yResolution, useEncoderValues,
                                          triggerInterval, mmind::eye::CoordinateUnit::Meter)
                   .data())),
           (texturedCloud.row_step * texturedCloud.height));
}

void callbackFunc(const mmind::eye::ProfileBatch& batch, void* pUser)
{
    auto* mechMindProfiler = static_cast<MechMindProfiler*>(pUser);
    if (mechMindProfiler)
        mechMindProfiler->handleCallbackBatch(batch);
}
} // namespace

MechMindProfiler::MechMindProfiler()
{
    ros::NodeHandle pnh("~");

    pnh.getParam("profiler_ip", profiler_ip);
    pnh.getParam("save_file", save_file);

    pub_intensity = nh.advertise<sensor_msgs::Image>("/mechmind_profiler/intensity_image", 1);
    pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind_profiler/depth_map", 1);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind_profiler/point_cloud", 1);
    pub_textured_pcl =
        nh.advertise<sensor_msgs::PointCloud2>("/mechmind_profiler/textured_point_cloud", 1);

    if (!findAndConnect(profiler))
        throw mmind::eye::ErrorStatus{mmind::eye::ErrorStatus::MMIND_STATUS_INVALID_DEVICE,
                                      "Profiler not found."};

    // Uncomment the following lines and comment the above if function to connect to a specific
    // profiler by its IP address. The IP address is set in the "start_profiler.launch" file as the
    // value of the "profiler_ip" argument.

    // mmind::eye::ErrorStatus status;
    // mmind::eye::ProfilerInfo info;
    // info.firmwareVersion = mmind::eye::Version("2.4.0");
    // info.ipAddress = profiler_ip;
    // info.port = 5577;
    // status = profiler.connect(info);
    // if (!status.isOK())
    // {
    //     throw status;
    // }
    // std::cout << "Connected to the profiler successfully." << std::endl;

    mmind::eye::ProfilerInfo profilerInfo;
    showError(profiler.getProfilerInfo(profilerInfo));
    printProfilerInfo(profilerInfo);

    auto status = profiler.registerAcquisitionCallback(callbackFunc, this);
    if (!status.isOK()) {
        throw status;
    }

    start_acquisition_service = nh.advertiseService(
        "start_acquisition", &MechMindProfiler::start_acquisition_callback, this);

    stop_acquisition_service =
        nh.advertiseService("stop_acquisition", &MechMindProfiler::stop_acquisition_callback, this);

    trigger_software_service =
        nh.advertiseService("trigger_software", &MechMindProfiler::trigger_software_callback, this);

    add_user_set_service =
        nh.advertiseService("add_user_set", &MechMindProfiler::add_user_set_callback, this);

    delete_user_set_service =
        nh.advertiseService("delete_user_set", &MechMindProfiler::delete_user_set_callback, this);

    profiler_info_service =
        nh.advertiseService("profiler_info", &MechMindProfiler::profiler_info_callback, this);

    get_all_user_sets_service = nh.advertiseService(
        "get_all_user_sets", &MechMindProfiler::get_all_user_sets_callback, this);

    get_current_user_set_service = nh.advertiseService(
        "get_current_user_set", &MechMindProfiler::get_current_user_set_callback, this);

    save_all_settings_to_user_sets_service =
        nh.advertiseService("save_all_settings_to_user_sets",
                            &MechMindProfiler::save_all_settings_to_user_sets_callback, this);

    set_current_user_set_service = nh.advertiseService(
        "set_current_user_set", &MechMindProfiler::set_current_user_set_callback, this);

    set_int_parameter_service = nh.advertiseService(
        "set_int_parameter", &MechMindProfiler::set_int_parameter_callback, this);

    get_int_parameter_service = nh.advertiseService(
        "get_int_parameter", &MechMindProfiler::get_int_parameter_callback, this);

    set_float_parameter_service = nh.advertiseService(
        "set_float_parameter", &MechMindProfiler::set_float_parameter_callback, this);

    get_float_parameter_service = nh.advertiseService(
        "get_float_parameter", &MechMindProfiler::get_float_parameter_callback, this);

    set_bool_parameter_service = nh.advertiseService(
        "set_bool_parameter", &MechMindProfiler::set_bool_parameter_callback, this);

    get_bool_parameter_service = nh.advertiseService(
        "get_bool_parameter", &MechMindProfiler::get_bool_parameter_callback, this);

    set_enum_parameter_service = nh.advertiseService(
        "set_enum_parameter", &MechMindProfiler::set_enum_parameter_callback, this);

    get_enum_parameter_service = nh.advertiseService(
        "get_enum_parameter", &MechMindProfiler::get_enum_parameter_callback, this);

    set_profile_roi_parameter_service = nh.advertiseService(
        "set_profile_roi_parameter", &MechMindProfiler::set_profile_roi_parameter_callback, this);

    get_profile_roi_parameter_service = nh.advertiseService(
        "get_profile_roi_parameter", &MechMindProfiler::get_profile_roi_parameter_callback, this);
}

void MechMindProfiler::handleCallbackBatch(const mmind::eye::ProfileBatch& batch)
{
    auto batchBackup = batch;
    if (!batchBackup.getErrorStatus().isOK()) {
        showError(batch.getErrorStatus());
        return;
    }
    if (batchBackup.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << batchBackup.validHeight() << "." << std::endl;
    publishIntensityImage(batchBackup.getIntensityImage());
    publishDepthMap(batchBackup.getDepthMap());
    publishPointClouds(batchBackup, profiler.currentUserSet());
}

void MechMindProfiler::publishIntensityImage(
    mmind::eye::ProfileBatch::IntensityImage&& intensityImage)
{
    cv::Mat intensity =
        cv::Mat(intensityImage.height(), intensityImage.width(), CV_8UC1, intensityImage.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = intensity;
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_profiler/intensity_image";
    ros_image.header.stamp = ros::Time::now();
    pub_intensity.publish(ros_image);
    if (!save_file)
        return;
    if (cv::imwrite("/tmp/intensity_image.png", intensity))
        std::cout << "The intensity image is saved to /tmp" << std::endl;
    else
        std::cerr << "Failed to save the intensity image." << std::endl;
}

void MechMindProfiler::publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_profiler/depth_map";
    ros_depth.header.stamp = ros::Time::now();
    pub_depth.publish(ros_depth);
    if (!save_file)
        return;
    if (cv::imwrite("/tmp/depth_map.tiff", depth))
        std::cout << "The depth map is saved to /tmp." << std::endl;
    else
        std::cerr << "Failed to save the depth map." << std::endl;
}

void MechMindProfiler::publishPointClouds(const mmind::eye::ProfileBatch& batch,
                                          const mmind::eye ::UserSet& userSet)
{
    if (batch.isEmpty())
        return;

    // Get the X-axis resolution
    double xResolution{};
    auto status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name,
                                        xResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    // Get the Y resolution
    double yResolution{};
    status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    int lineScanTriggerSource{};
    status = userSet.getEnumValue(mmind::eye::trigger_settings::LineScanTriggerSource::name,
                                  lineScanTriggerSource);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    bool useEncoderValues =
        lineScanTriggerSource ==
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder);

    int triggerInterval{};
    status = userSet.getIntValue(mmind::eye::trigger_settings::EncoderTriggerInterval::name,
                                 triggerInterval);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    sensor_msgs::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = "mechmind_profiler/point_cloud";
    ros_cloud.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 ros_textured_cloud;
    ros_textured_cloud.header.frame_id = "mechmind_profiler/textured_point_cloud";
    ros_textured_cloud.header.stamp = ros::Time::now();
    convertToROSMsg(batch, xResolution, yResolution, useEncoderValues, triggerInterval, ros_cloud,
                    ros_textured_cloud);
    pub_pcl.publish(ros_cloud);
    pub_textured_pcl.publish(ros_textured_cloud);
}

bool MechMindProfiler::start_acquisition_callback(
    mecheye_profiler_ros_interface::StartAcquisition::Request& req,
    mecheye_profiler_ros_interface::StartAcquisition::Response& res)
{
    const auto status = profiler.startAcquisition();
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::stop_acquisition_callback(
    mecheye_profiler_ros_interface::StopAcquisition::Request& req,
    mecheye_profiler_ros_interface::StopAcquisition::Response& res)
{
    const auto status = profiler.stopAcquisition();
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::trigger_software_callback(
    mecheye_profiler_ros_interface::TriggerSoftware::Request& req,
    mecheye_profiler_ros_interface::TriggerSoftware::Response& res)
{
    const auto status = profiler.triggerSoftware();
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::profiler_info_callback(
    mecheye_profiler_ros_interface::ProfilerInfo::Request& req,
    mecheye_profiler_ros_interface::ProfilerInfo::Response& res)
{
    mmind::eye::ProfilerInfo profilerInfo;
    const auto status = profiler.getProfilerInfo(profilerInfo);
    showError(status);
    res.model = profilerInfo.model.c_str();
    res.controller_sn = profilerInfo.controllerSN.c_str();
    res.sensor_sn = profilerInfo.sensorSN.c_str();
    res.hardware_version = profilerInfo.hardwareVersion.toString();
    res.firmware_version = profilerInfo.firmwareVersion.toString();
    res.ip_address = profilerInfo.ipAddress.c_str();
    res.subnet_mask = profilerInfo.subnetMask.c_str();
    res.ip_assignment_method = ipAssignmentMethodToString(profilerInfo.ipAssignmentMethod);
    res.port = profilerInfo.port;
    return true;
}

bool MechMindProfiler::get_all_user_sets_callback(
    mecheye_profiler_ros_interface::GetAllUserSets::Request& req,
    mecheye_profiler_ros_interface::GetAllUserSets::Response& res)
{
    std::vector<std::string> sequence;
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.getAllUserSetNames(sequence);
    showError(status);
    res.sequence = sequence;
    return true;
}

bool MechMindProfiler::get_current_user_set_callback(
    mecheye_profiler_ros_interface::GetCurrentUserSet::Request& req,
    mecheye_profiler_ros_interface::GetCurrentUserSet::Response& res)
{
    std::string value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getName(value);
    showError(status);
    res.value = value.c_str();
    return true;
}

bool MechMindProfiler::set_current_user_set_callback(
    mecheye_profiler_ros_interface::SetCurrentUserSet::Request& req,
    mecheye_profiler_ros_interface::SetCurrentUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.selectUserSet(req.value.c_str());
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::add_user_set_callback(
    mecheye_profiler_ros_interface::AddUserSet::Request& req,
    mecheye_profiler_ros_interface::AddUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.addUserSet({req.value.c_str()});
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::delete_user_set_callback(
    mecheye_profiler_ros_interface::DeleteUserSet::Request& req,
    mecheye_profiler_ros_interface::DeleteUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.deleteUserSet({req.value.c_str()});
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::save_all_settings_to_user_sets_callback(
    mecheye_profiler_ros_interface::SaveAllSettingsToUserSets::Request& req,
    mecheye_profiler_ros_interface::SaveAllSettingsToUserSets::Response& res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.saveAllParametersToDevice();
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::get_profile_roi_parameter_callback(
    mecheye_profiler_ros_interface::GetProfileROIParameter::Request& req,
    mecheye_profiler_ros_interface::GetProfileROIParameter::Response& res)
{
    mmind::eye::ProfileROI roi;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getProfileRoiValue(req.name, roi);
    showError(status);
    res.x_axis_center = roi.xAxisCenter;
    res.width = roi.width;
    res.height = roi.height;
    return true;
}

bool MechMindProfiler::set_profile_roi_parameter_callback(
    mecheye_profiler_ros_interface::SetProfileROIParameter::Request& req,
    mecheye_profiler_ros_interface::SetProfileROIParameter::Response& res)
{
    mmind::eye::ProfileROI roi{req.x_axis_center, req.width, req.height};
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setProfileRoiValue(req.name, roi);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::get_int_parameter_callback(
    mecheye_profiler_ros_interface::GetIntParameter::Request& req,
    mecheye_profiler_ros_interface::GetIntParameter::Response& res)
{
    int value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getIntValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindProfiler::set_int_parameter_callback(
    mecheye_profiler_ros_interface::SetIntParameter::Request& req,
    mecheye_profiler_ros_interface::SetIntParameter::Response& res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setIntValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::get_bool_parameter_callback(
    mecheye_profiler_ros_interface::GetBoolParameter::Request& req,
    mecheye_profiler_ros_interface::GetBoolParameter::Response& res)
{
    bool value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getBoolValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindProfiler::set_bool_parameter_callback(
    mecheye_profiler_ros_interface::SetBoolParameter::Request& req,
    mecheye_profiler_ros_interface::SetBoolParameter::Response& res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setBoolValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::get_enum_parameter_callback(
    mecheye_profiler_ros_interface::GetEnumParameter::Request& req,
    mecheye_profiler_ros_interface::GetEnumParameter::Response& res)
{
    std::string value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getEnumValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindProfiler::set_enum_parameter_callback(
    mecheye_profiler_ros_interface::SetEnumParameter::Request& req,
    mecheye_profiler_ros_interface::SetEnumParameter::Response& res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setEnumValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindProfiler::get_float_parameter_callback(
    mecheye_profiler_ros_interface::GetFloatParameter::Request& req,
    mecheye_profiler_ros_interface::GetFloatParameter::Response& res)
{
    double value = -1;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindProfiler::set_float_parameter_callback(
    mecheye_profiler_ros_interface::SetFloatParameter::Request& req,
    mecheye_profiler_ros_interface::SetFloatParameter::Response& res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}
