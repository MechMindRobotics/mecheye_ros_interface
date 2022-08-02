#include <MechMindCamera.h>
#include <SampleUtil.h>
#include <OpenCVUtil.h>
#include <PclUtil.h>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/String.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mecheye_ros_interface
{

    MechMindCamera::MechMindCamera()
    {
        ros::NodeHandle pnh("~");

        pnh.getParam("camera_ip", camera_ip);
        pnh.getParam("save_file", save_file);
        pnh.getParam("use_external_intri", use_external_intri);
        pnh.getParam("fx", fx);
        pnh.getParam("fy", fy);
        pnh.getParam("u", u);
        pnh.getParam("v", v);
        pnh.getParam("tf_x", x);
        pnh.getParam("tf_y", y);
        pnh.getParam("tf_z", z);
        pnh.getParam("tf_qx", qx);
        pnh.getParam("tf_qy", qy);
        pnh.getParam("tf_qz", qz);
        pnh.getParam("tf_qw", qw);

        pub_color = nh.advertise<sensor_msgs::Image>("/mechmind/color_image", 1);
        pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind/depth_image", 1);
        pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/point_cloud", 1);
        pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/color_point_cloud", 1);
        pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/mechmind/camera_info", 1);

        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped pclTransform;
        pclTransform.header.stamp = ros::Time::now();
        pclTransform.header.frame_id = "map";
        pclTransform.child_frame_id = "/mechmind_camera/point_cloud";
        pclTransform.transform.translation.x = x;
        pclTransform.transform.translation.y = y;
        pclTransform.transform.translation.z = z;
        pclTransform.transform.rotation.x = qx;
        pclTransform.transform.rotation.y = qy;
        pclTransform.transform.rotation.z = qz;
        pclTransform.transform.rotation.w = qw;
        geometry_msgs::TransformStamped colorPclTransform;
        colorPclTransform.header.stamp = ros::Time::now();
        colorPclTransform.header.frame_id = "map";
        colorPclTransform.child_frame_id = "/mechmind_camera/color_point_cloud";
        colorPclTransform.transform.translation.x = x;
        colorPclTransform.transform.translation.y = y;
        colorPclTransform.transform.translation.z = z;
        colorPclTransform.transform.rotation.x = qx;
        colorPclTransform.transform.rotation.y = qy;
        colorPclTransform.transform.rotation.z = qz;
        colorPclTransform.transform.rotation.w = qw;
        static_broadcaster.sendTransform(pclTransform);
        static_broadcaster.sendTransform(colorPclTransform);

        if (!findAndConnect(device))
            return;

        // Uncomment the following lines to connect a camera with ip inside .launch file

        // mmind::api::ErrorStatus status;
        // mmind::api::MechEyeDeviceInfo info;
        // info.firmwareVersion = "1.5.2";
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
            intri.textureCameraIntri.cameraMatrix[0] = fx;
            intri.textureCameraIntri.cameraMatrix[1] = fy;
            intri.textureCameraIntri.cameraMatrix[2] = u;
            intri.textureCameraIntri.cameraMatrix[3] = v;
            intri.depthCameraIntri.cameraMatrix[0] = fx;
            intri.depthCameraIntri.cameraMatrix[1] = fy;
            intri.depthCameraIntri.cameraMatrix[2] = u;
            intri.depthCameraIntri.cameraMatrix[3] = v;
        }
        else
        {
            showError(device.getDeviceIntri(intri));
        }

        add_user_set_service = nh.advertiseService("add_user_set", &MechMindCamera::add_user_set_callback, this);
        capture_color_map_service =
            nh.advertiseService("capture_color_map", &MechMindCamera::capture_color_map_callback, this);
        capture_color_point_cloud_service = nh.advertiseService("capture_color_point_cloud",
                                                                &MechMindCamera::capture_color_point_cloud_callback, this);
        capture_depth_map_service =
            nh.advertiseService("capture_depth_map", &MechMindCamera::capture_depth_map_callback, this);
        capture_point_cloud_service =
            nh.advertiseService("capture_point_cloud", &MechMindCamera::capture_point_cloud_callback, this);
        delete_user_set_service = nh.advertiseService("delete_user_set", &MechMindCamera::delete_user_set_callback, this);
        device_info_service = nh.advertiseService("device_info", &MechMindCamera::device_info_callback, this);
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
        get_uhp_capture_mode_service =
            nh.advertiseService("get_uhp_capture_mode", &MechMindCamera::get_uhp_capture_mode_callback, this);
        get_uhp_fringe_coding_mode_service =
            nh.advertiseService("get_uhp_fringe_coding_mode", &MechMindCamera::get_uhp_fringe_coding_mode_callback, this);
        get_uhp_settings_service =
            nh.advertiseService("get_uhp_settings", &MechMindCamera::get_uhp_settings_callback, this);
        save_all_settings_to_user_sets_service = nh.advertiseService(
            "save_all_settings_to_user_sets", &MechMindCamera::save_all_settings_to_user_sets_callback, this);
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
        set_uhp_capture_mode_service =
            nh.advertiseService("set_uhp_capture_mode", &MechMindCamera::set_uhp_capture_mode_callback, this);
        set_uhp_fringe_coding_mode_service =
            nh.advertiseService("set_uhp_fringe_coding_mode", &MechMindCamera::set_uhp_fringe_coding_mode_callback, this);
        set_uhp_settings_service =
            nh.advertiseService("set_uhp_settings", &MechMindCamera::set_uhp_settings_callback, this);
    }

    void MechMindCamera::publishColorMap(mmind::api::ColorMap &colorMap)
    {
        cv::Mat color = cv::Mat(colorMap.height(), colorMap.width(), CV_8UC3, colorMap.data());
        cv_bridge::CvImage cv_image;
        cv_image.image = color;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        ros_image.header.frame_id = "mechmind_camera/color_map";
        ros_image.header.stamp = ros::Time::now();
        pub_color.publish(ros_image);
        publishCameraInfo(ros_image.header, colorMap.width(), colorMap.height());
        if (save_file)
            saveMap(colorMap, "/tmp/mechmind_color.png");
    }

    void MechMindCamera::publishDepthMap(mmind::api::DepthMap &depthMap)
    {
        cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
        cv_bridge::CvImage cv_depth;
        cv_depth.image = depth;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        sensor_msgs::Image ros_depth;
        cv_depth.toImageMsg(ros_depth);
        ros_depth.header.frame_id = "mechmind_camera/depth_map";
        ros_depth.header.stamp = ros::Time::now();
        pub_depth.publish(ros_depth);
        publishCameraInfo(ros_depth.header, depthMap.width(), depthMap.height());
        if (save_file)
            saveMap(depthMap, "/tmp/mechmind_depth.tiff");
    }

    void MechMindCamera::publishPointCloud(mmind::api::PointXYZMap &pointXYZMap)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud(pointXYZMap.width(), pointXYZMap.height());
        toPCL(cloud, pointXYZMap);
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "mechmind_camera/point_cloud";
        ros_cloud.header.stamp = ros::Time::now();
        pub_pcl.publish(ros_cloud);
        publishCameraInfo(ros_cloud.header, pointXYZMap.width(), pointXYZMap.height());
        if (save_file)
            savePLY(pointXYZMap, "/tmp/mechmind_cloud.ply");
    }

    void MechMindCamera::publishColorPointCloud(mmind::api::PointXYZBGRMap &pointXYZBGRMap)
    {
        pcl::PointCloud<pcl::PointXYZRGB> color_cloud(pointXYZBGRMap.width(), pointXYZBGRMap.height());
        toPCL(color_cloud, pointXYZBGRMap);
        sensor_msgs::PointCloud2 ros_color_cloud;
        pcl::toROSMsg(color_cloud, ros_color_cloud);
        ros_color_cloud.header.frame_id = "mechmind_camera/color_point_cloud";
        ros_color_cloud.header.stamp = ros::Time::now();
        pub_pcl_color.publish(ros_color_cloud);
        publishCameraInfo(ros_color_cloud.header, pointXYZBGRMap.width(), pointXYZBGRMap.height());
        if (save_file)
            savePLY(pointXYZBGRMap, "/tmp/mechmind_color_cloud.ply");
    }

    void MechMindCamera::publishCameraInfo(const std_msgs::Header &header, int width, int height)
    {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header = header;
        camera_info.height = height;
        camera_info.width = width;
        camera_info.distortion_model = "plumb_bob";

        camera_info.D = std::vector<double>(intri.textureCameraIntri.distortion, intri.textureCameraIntri.distortion + 5);

        std::vector<double> K{intri.textureCameraIntri.cameraMatrix[0],
                              0.0,
                              intri.textureCameraIntri.cameraMatrix[2],
                              0.0,
                              intri.textureCameraIntri.cameraMatrix[1],
                              intri.textureCameraIntri.cameraMatrix[3],
                              0.0,
                              0.0,
                              1.0};
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info.K[i] = K[i];
        }

        std::vector<double> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info.R[i] = R[i];
        }

        std::vector<double> P{intri.textureCameraIntri.cameraMatrix[0],
                              0.0,
                              intri.textureCameraIntri.cameraMatrix[2],
                              0.0,
                              0.0,
                              intri.textureCameraIntri.cameraMatrix[1],
                              intri.textureCameraIntri.cameraMatrix[3],
                              0.0,
                              0.0,
                              0.0,
                              1.0,
                              0.0};
        for (size_t i = 0; i < 12; ++i)
        {
            camera_info.P[i] = P[i];
        }
        pub_camera_info.publish(camera_info);
    }

    bool MechMindCamera::add_user_set_callback(AddUserSet::Request &req, AddUserSet::Response &res)
    {
        mmind::api::ErrorStatus status = device.addUserSet({req.value.c_str()});
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::capture_color_map_callback(CaptureColorMap::Request &req, CaptureColorMap::Response &res)
    {
        mmind::api::ColorMap colorMap;
        mmind::api::ErrorStatus status = device.captureColorMap(colorMap);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        publishColorMap(colorMap);
        return true;
    }

    bool MechMindCamera::capture_color_point_cloud_callback(CaptureColorPointCloud::Request &req,
                                                            CaptureColorPointCloud::Response &res)
    {
        mmind::api::PointXYZBGRMap pointXYZBGRMap;
        mmind::api::ErrorStatus status = device.capturePointXYZBGRMap(pointXYZBGRMap);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        publishColorPointCloud(pointXYZBGRMap);
        return true;
    }

    bool MechMindCamera::capture_depth_map_callback(CaptureDepthMap::Request &req, CaptureDepthMap::Response &res)
    {
        mmind::api::DepthMap depthMap;
        mmind::api::ErrorStatus status = device.captureDepthMap(depthMap);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        publishDepthMap(depthMap);
        return true;
    }

    bool MechMindCamera::capture_point_cloud_callback(CapturePointCloud::Request &req, CapturePointCloud::Response &res)
    {
        mmind::api::PointXYZMap pointXYZMap;
        mmind::api::ErrorStatus status = device.capturePointXYZMap(pointXYZMap);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        publishPointCloud(pointXYZMap);
        return true;
    }

    bool MechMindCamera::delete_user_set_callback(DeleteUserSet::Request &req, DeleteUserSet::Response &res)
    {
        mmind::api::ErrorStatus status = device.deleteUserSet({req.value.c_str()});
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::device_info_callback(DeviceInfo::Request &req, DeviceInfo::Response &res)
    {
        mmind::api::MechEyeDeviceInfo deviceInfo;
        mmind::api::ErrorStatus status = device.getDeviceInfo(deviceInfo);
        showError(status);
        res.model = deviceInfo.model.c_str();
        res.id = deviceInfo.id.c_str();
        res.hardwareVersion = deviceInfo.hardwareVersion.c_str();
        res.firmwareVersion = deviceInfo.firmwareVersion.c_str();
        res.ipAddress = deviceInfo.ipAddress.c_str();
        res.port = deviceInfo.port;
        return status.isOK();
    }

    bool MechMindCamera::get_2d_expected_gray_value_callback(Get2DExpectedGrayValue::Request &req,
                                                             Get2DExpectedGrayValue::Response &res)
    {
        int value;
        mmind::api::ErrorStatus status = device.getScan2DExpectedGrayValue(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }
    bool MechMindCamera::get_2d_exposure_mode_callback(Get2DExposureMode::Request &req, Get2DExposureMode::Response &res)
    {
        mmind::api::Scanning2DSettings::Scan2DExposureMode mode;
        mmind::api::ErrorStatus status = device.getScan2DExposureMode(mode);
        showError(status);
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

    bool MechMindCamera::get_2d_exposure_sequence_callback(Get2DExposureSequence::Request &req,
                                                           Get2DExposureSequence::Response &res)
    {
        std::vector<double> sequence;
        mmind::api::ErrorStatus status = device.getScan2DHDRExposureSequence(sequence);
        showError(status);
        res.sequence = sequence;
        return status.isOK();
    }

    bool MechMindCamera::get_2d_exposure_time_callback(Get2DExposureTime::Request &req, Get2DExposureTime::Response &res)
    {
        double value = -1;
        mmind::api::ErrorStatus status = device.getScan2DExposureTime(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_2d_roi_callback(Get2DROI::Request &req, Get2DROI::Response &res)
    {
        mmind::api::ROI roi;
        mmind::api::ErrorStatus status = device.getScan2DROI(roi);
        showError(status);
        res.x = roi.x;
        res.y = roi.y;
        res.width = roi.width;
        res.height = roi.height;
        return status.isOK();
    }

    bool MechMindCamera::get_2d_sharpen_factor_callback(Get2DSharpenFactor::Request &req, Get2DSharpenFactor::Response &res)
    {
        double value = -1;
        mmind::api::ErrorStatus status = device.getScan2DSharpenFactor(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_2d_tone_mapping_callback(Get2DToneMappingEnable::Request &req,
                                                      Get2DToneMappingEnable::Response &res)
    {
        bool value;
        mmind::api::ErrorStatus status = device.getScan2DToneMappingEnable(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_3d_exposure_callback(Get3DExposure::Request &req, Get3DExposure::Response &res)
    {
        std::vector<double> sequence;
        mmind::api::ErrorStatus status = device.getScan3DExposure(sequence);
        showError(status);
        res.sequence = sequence;
        return status.isOK();
    }

    bool MechMindCamera::get_3d_gain_callback(Get3DGain::Request &req, Get3DGain::Response &res)
    {
        double value = -1;
        mmind::api::ErrorStatus status = device.getScan3DGain(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_3d_roi_callback(Get3DROI::Request &req, Get3DROI::Response &res)
    {
        mmind::api::ROI roi;
        mmind::api::ErrorStatus status = device.getScan3DROI(roi);
        showError(status);
        res.x = roi.x;
        res.y = roi.y;
        res.width = roi.width;
        res.height = roi.height;
        return status.isOK();
    }

    bool MechMindCamera::get_all_user_sets_callback(GetAllUserSets::Request &req, GetAllUserSets::Response &res)
    {
        std::vector<std::string> sequence;
        mmind::api::ErrorStatus status = device.getAllUserSets(sequence);
        showError(status);
        // std::vector<char*> charPtrSequence;
        // for (auto& s : sequence)
        // {
        //     charPtrSequence.emplace_back(s.c_str());
        // }
        // res.sequence = charPtrSequence;
        res.sequence = sequence;
        return status.isOK();
    }

    bool MechMindCamera::get_cloud_outlier_filter_mode_callback(GetCloudOutlierFilterMode::Request &req,
                                                                GetCloudOutlierFilterMode ::Response &res)
    {
        mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode mode;
        mmind::api::ErrorStatus status = device.getCloudOutlierFilterMode(mode);
        showError(status);
        switch (mode)
        {
        case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Off:
            res.value = "Off";
            break;

        case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal:
            res.value = "Normal";
            break;

        case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Weak:
            res.value = "Weak";
            break;

        default:
            res.value = "";
            break;
        }
        return status.isOK();
    }

    bool MechMindCamera::get_cloud_smooth_mode_callback(GetCloudSmoothMode::Request &req, GetCloudSmoothMode::Response &res)
    {
        mmind::api::PointCloudProcessingSettings::CloudSmoothMode mode;
        mmind::api::ErrorStatus status = device.getCloudSmoothMode(mode);
        showError(status);
        switch (mode)
        {
        case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Off:
            res.value = "Off";
            break;

        case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal:
            res.value = "Normal";
            break;

        case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Weak:
            res.value = "Weak";
            break;

        case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Strong:
            res.value = "Strong";
            break;

        default:
            res.value = "";
            break;
        }
        return status.isOK();
    }

    bool MechMindCamera::get_current_user_set_callback(GetCurrentUserSet::Request &req, GetCurrentUserSet::Response &res)
    {
        std::string value;
        mmind::api::ErrorStatus status = device.getCurrentUserSet(value);
        showError(status);
        res.value = value.c_str();
        return status.isOK();
    }

    bool MechMindCamera::get_depth_range_callback(GetDepthRange::Request &req, GetDepthRange::Response &res)
    {
        mmind::api::DepthRange depthRange;
        mmind::api::ErrorStatus status = device.getDepthRange(depthRange);
        showError(status);
        res.lower = depthRange.lower;
        res.upper = depthRange.upper;
        return status.isOK();
    }

    bool MechMindCamera::get_fringe_contrast_threshold_callback(GetFringeContrastThreshold::Request &req,
                                                                GetFringeContrastThreshold::Response &res)
    {
        int value;
        mmind::api::ErrorStatus status = device.getFringeContrastThreshold(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_fringe_min_threshold_callback(GetFringeMinThreshold::Request &req,
                                                           GetFringeMinThreshold::Response &res)
    {
        int value;
        mmind::api::ErrorStatus status = device.getFringeMinThreshold(value);
        showError(status);
        res.value = value;
        return status.isOK();
    }

    bool MechMindCamera::get_laser_settings_callback(GetLaserSettings::Request &req, GetLaserSettings::Response &res)
    {
        mmind::api::LaserSettings laserSettings;
        mmind::api::ErrorStatus status = device.getLaserSettings(laserSettings);
        showError(status);
        switch (laserSettings.FringeCodingMode)
        {
        case 0:
            res.fringeCodingMode = "Fast";
            break;

        case 1:
            res.fringeCodingMode = "Accurate";
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

    bool MechMindCamera::get_uhp_settings_callback(GetUhpSettings::Request &req, GetUhpSettings::Response &res)
    {
        mmind::api::UhpSettings uhpSettings;
        mmind::api::ErrorStatus status = device.getUhpSettings(uhpSettings);
        showError(status);
        switch (uhpSettings.CaptureMode)
        {
        case mmind::api::UhpSettings::UhpCaptureMode::Camera1:
            res.capture_mode = "Camera1";
            break;

        case mmind::api::UhpSettings::UhpCaptureMode::Camera2:
            res.capture_mode = "Camera2";
            break;

        case mmind::api::UhpSettings::UhpCaptureMode::Merge:
            res.capture_mode = "Merge";
            break;
        
        default:
            res.capture_mode = "";
            break;
        }
        switch (uhpSettings.FringeCodingMode)
        {
        case mmind::api::UhpSettings::UhpFringeCodingMode::Fast:
            res.fringe_coding_mode = "Fast";
            break;

        case mmind::api::UhpSettings::UhpFringeCodingMode::Accurate:
            res.fringe_coding_mode = "Accurate";
            break;

        default:
            res.fringe_coding_mode = "";
            break;
        }
        return status.isOK();
    }

    bool MechMindCamera::get_uhp_capture_mode_callback(GetUhpCaptureMode::Request &req, GetUhpCaptureMode::Response &res)
    {
        mmind::api::UhpSettings::UhpCaptureMode mode;
        mmind::api::ErrorStatus status = device.getUhpCaptureMode(mode);
        showError(status);
        switch (mode)
        {
        case mmind::api::UhpSettings::UhpCaptureMode::Camera1:
            res.capture_mode = "Camera1";
            break;

        case mmind::api::UhpSettings::UhpCaptureMode::Camera2:
            res.capture_mode = "Camera2";
            break;

        case mmind::api::UhpSettings::UhpCaptureMode::Merge:
            res.capture_mode = "Merge";
            break;
        
        default:
            res.capture_mode = "";
            break;
        }
        return status.isOK();
    }

    bool MechMindCamera::get_uhp_fringe_coding_mode_callback(GetUhpFringeCodingMode::Request &req, GetUhpFringeCodingMode::Response &res)
    {
        mmind::api::UhpSettings::UhpFringeCodingMode mode;
        mmind::api::ErrorStatus status = device.getUhpFringeCodingMode(mode);
        showError(status);
        switch (mode)
        {
        case mmind::api::UhpSettings::UhpFringeCodingMode::Fast:
            res.fringe_coding_mode = "Fast";
            break;

        case mmind::api::UhpSettings::UhpFringeCodingMode::Accurate:
            res.fringe_coding_mode = "Accurate";
            break;
        
        default:
            res.fringe_coding_mode = "";
            break;
        }
        return status.isOK();
    }

    bool MechMindCamera::save_all_settings_to_user_sets_callback(SaveAllSettingsToUserSets::Request &req,
                                                                 SaveAllSettingsToUserSets::Response &res)
    {
        return device.saveAllSettingsToUserSets().isOK();
    }

    bool MechMindCamera::set_2d_expected_gray_value_callback(Set2DExpectedGrayValue::Request &req,
                                                             Set2DExpectedGrayValue::Response &res)
    {
        mmind::api::ErrorStatus status = device.setScan2DExpectedGrayValue(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_exposure_mode_callback(Set2DExposureMode::Request &req, Set2DExposureMode::Response &res)
    {
        int mode;
        if (req.value == "Timed")
            mode = 0;
        else if (req.value == "Auto")
            mode = 1;
        else if (req.value == "HDR")
            mode = 2;
        else if (req.value == "Flash")
            mode = 3;
        else
        {
            res.errorCode = mmind::api::ErrorStatus::ErrorCode::MMIND_STATUS_PARAMETER_SET_ERROR;
            res.errorDescription = "Invalid parameter. Valid choices include 'Timed', 'Auto', 'HDR', and 'Flash'.";
            return true;
        }
        mmind::api::ErrorStatus status = device.setScan2DExposureMode(mmind::api::Scanning2DSettings::Scan2DExposureMode(mode));
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_exposure_sequence_callback(Set2DExposureSequence::Request &req,
                                                           Set2DExposureSequence::Response &res)
    {
        std::vector<double> sequence(begin(req.sequence), end(req.sequence));
        mmind::api::ErrorStatus status = device.setScan2DHDRExposureSequence(sequence);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_exposure_time_callback(Set2DExposureTime::Request &req, Set2DExposureTime::Response &res)
    {
        mmind::api::ErrorStatus status = device.setScan2DExposureTime(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_roi_callback(Set2DROI::Request &req, Set2DROI::Response &res)
    {
        mmind::api::ROI roi{req.x, req.y, req.width, req.height};
        mmind::api::ErrorStatus status = device.setScan2DROI(roi);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_sharpen_factor_callback(Set2DSharpenFactor::Request &req, Set2DSharpenFactor::Response &res)
    {
        mmind::api::ErrorStatus status = device.setScan2DSharpenFactor(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_2d_tone_mapping_callback(Set2DToneMappingEnable::Request &req,
                                                      Set2DToneMappingEnable::Response &res)
    {
        mmind::api::ErrorStatus status = device.setScan2DToneMappingEnable(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_3d_exposure_callback(Set3DExposure::Request &req, Set3DExposure::Response &res)
    {
        std::vector<double> sequence(begin(req.sequence), end(req.sequence));
        mmind::api::ErrorStatus status = device.setScan3DExposure(sequence);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_3d_gain_callback(Set3DGain::Request &req, Set3DGain::Response &res)
    {
        mmind::api::ErrorStatus status = device.setScan3DGain(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_3d_roi_callback(Set3DROI::Request &req, Set3DROI::Response &res)
    {
        mmind::api::ROI roi{req.x, req.y, req.width, req.height};
        mmind::api::ErrorStatus status = device.setScan3DROI(roi);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_cloud_outlier_filter_mode_callback(SetCloudOutlierFilterMode::Request &req,
                                                                SetCloudOutlierFilterMode ::Response &res)
    {
        mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode mode;
        if (req.value == "Off")
            mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Off;
        else if (req.value == "Normal")
            mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal;
        else if (req.value == "Weak")
            mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Weak;
        else
        {
            res.errorCode = mmind::api::ErrorStatus::ErrorCode::MMIND_STATUS_PARAMETER_SET_ERROR;
            res.errorDescription = "Invalid parameter. Valid choices include '!!str Off', 'Normal', and 'Weak'.";
            return true;
        }
        mmind::api::ErrorStatus status = device.setCloudOutlierFilterMode(mode);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_cloud_smooth_mode_callback(SetCloudSmoothMode::Request &req, SetCloudSmoothMode::Response &res)
    {
        mmind::api::PointCloudProcessingSettings::CloudSmoothMode mode;
        if (req.value == "Off")
            mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Off;
        else if (req.value == "Normal")
            mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal;
        else if (req.value == "Weak")
            mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Weak;
        else if (req.value == "Strong")
            mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Strong;
        else
        {
            res.errorCode = mmind::api::ErrorStatus::ErrorCode::MMIND_STATUS_PARAMETER_SET_ERROR;
            res.errorDescription = "Invalid parameter. Valid choices include '!!str Off', 'Normal', 'Weak', and 'Strong'.";
            return true;
        }
        mmind::api::ErrorStatus status = device.setCloudSmoothMode(mode);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_current_user_set_callback(SetCurrentUserSet::Request &req, SetCurrentUserSet::Response &res)
    {
        mmind::api::ErrorStatus status = device.setCurrentUserSet({req.value.c_str()});
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_depth_range_callback(SetDepthRange::Request &req, SetDepthRange::Response &res)
    {
        mmind::api::DepthRange depthRange{req.lower, req.upper};
        mmind::api::ErrorStatus status = device.setDepthRange(depthRange);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_fringe_contrast_threshold_callback(SetFringeContrastThreshold::Request &req,
                                                                SetFringeContrastThreshold::Response &res)
    {
        mmind::api::ErrorStatus status = device.setFringeContrastThreshold(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_fringe_min_threshold_callback(SetFringeMinThreshold::Request &req,
                                                           SetFringeMinThreshold::Response &res)
    {
        mmind::api::ErrorStatus status = device.setFringeMinThreshold(req.value);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_laser_settings_callback(SetLaserSettings::Request &req, SetLaserSettings::Response &res)
    {
        mmind::api::LaserSettings laserSettings{req.fringeCodingMode == "Fast" ? mmind::api::LaserSettings::LaserFringeCodingMode::Fast : mmind::api::LaserSettings::LaserFringeCodingMode::Accurate,
                                                req.frameRangeStart, req.frameRangeEnd, req.framePartitionCount, req.powerLevel};
        mmind::api::ErrorStatus status = device.setLaserSettings(laserSettings);
        showError(status);
        res.errorCode = status.errorCode;
        res.errorDescription = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_uhp_settings_callback(SetUhpSettings::Request &req, SetUhpSettings::Response &res)
    {
        mmind::api::UhpSettings uhpSettings{
            req.capture_mode == "Camera1" ? mmind::api::UhpSettings::UhpCaptureMode::Camera1 : (req.capture_mode == "Camera2" ? mmind::api::UhpSettings::UhpCaptureMode::Camera2 : mmind::api::UhpSettings::UhpCaptureMode::Merge),
            req.fringe_coding_mode == "Fast" ? mmind::api::UhpSettings::UhpFringeCodingMode::Fast : mmind::api::UhpSettings::UhpFringeCodingMode::Accurate
            };
        mmind::api::ErrorStatus status = device.setUhpSettings(uhpSettings);
        showError(status);
        res.error_code = status.errorCode;
        res.error_description = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_uhp_capture_mode_callback(SetUhpCaptureMode::Request &req, SetUhpCaptureMode::Response &res)
    {
        mmind::api::UhpSettings::UhpCaptureMode mode;
        if (req.capture_mode == "Camera1")
            mode = mmind::api::UhpSettings::UhpCaptureMode::Camera1;
        else if (req.capture_mode == "Camera2")
            mode = mmind::api::UhpSettings::UhpCaptureMode::Camera2;
        else if (req.capture_mode == "Merge")
            mode = mmind::api::UhpSettings::UhpCaptureMode::Merge;
        else
        {
            res.error_code = -4;
            res.error_description = "Invalid parameter";
            return false;
        }
        mmind::api::ErrorStatus status = device.setUhpCaptureMode(mode);
        showError(status);
        res.error_code = status.errorCode;
        res.error_description = status.errorDescription.c_str();
        return true;
    }

    bool MechMindCamera::set_uhp_fringe_coding_mode_callback(SetUhpFringeCodingMode::Request &req, SetUhpFringeCodingMode::Response &res)
    {
        mmind::api::UhpSettings::UhpFringeCodingMode mode;
        if (req.fringe_coding_mode == "Fast")
            mode = mmind::api::UhpSettings::UhpFringeCodingMode::Fast;
        else if (req.fringe_coding_mode == "Accurate")
            mode = mmind::api::UhpSettings::UhpFringeCodingMode::Accurate;
        else
        {
            res.error_code = -4;
            res.error_description = "Invalid parameter";
            return false;
        }
        mmind::api::ErrorStatus status = device.setUhpFringeCodingMode(mode);
        showError(status);
        res.error_code = status.errorCode;
        res.error_description = status.errorDescription.c_str();
        return true;
    }
} // namespace mecheye_ros_interface
