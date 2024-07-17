#include <MechMindCamera.h>
#include <area_scan_3d_camera/api_util.h>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <std_msgs/String.h>

namespace {

sensor_msgs::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype,
                                         uint32_t count)
{
    sensor_msgs::PointField point_field;
    point_field.name = name;
    point_field.offset = offset;
    point_field.datatype = datatype;
    point_field.count = count;
    return point_field;
}

void convertToROSMsg(const mmind::eye::TexturedPointCloud& texturedPointCloud,
                     sensor_msgs::PointCloud2& cloud)
{
    cloud.height = texturedPointCloud.height();
    cloud.width = texturedPointCloud.width();
    cloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(mmind::eye::PointXYZBGR);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    cloud.fields.reserve(4);
    cloud.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZBGR, x),
                                            sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZBGR, y),
                                            sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZBGR, z),
                                            sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("rgb", offsetof(mmind::eye::PointXYZBGR, rgb),
                                            sensor_msgs::PointField::FLOAT32, 1));

    memcpy(
        cloud.data.data(),
        reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZBGR*>(texturedPointCloud.data())),
        (cloud.row_step * cloud.height));
}

void convertToROSMsg(const mmind::eye::PointCloud& pointCloud, sensor_msgs::PointCloud2& cloud)
{
    cloud.height = pointCloud.height();
    cloud.width = pointCloud.width();
    cloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(mmind::eye::PointXYZ);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    cloud.fields.reserve(3);
    cloud.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZ, x),
                                            sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZ, y),
                                            sensor_msgs::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZ, z),
                                            sensor_msgs::PointField::FLOAT32, 1));
    memcpy(cloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(pointCloud.data())),
           (cloud.row_step * cloud.height));
}

} // namespace

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

    pub_color = nh.advertise<sensor_msgs::Image>("/mechmind/color_image", 1);
    pub_color_left = nh.advertise<sensor_msgs::Image>("/mechmind/stereo_color_image_left", 1);
    pub_color_right = nh.advertise<sensor_msgs::Image>("/mechmind/stereo_color_image_right", 1);
    pub_depth = nh.advertise<sensor_msgs::Image>("/mechmind/depth_map", 1);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/point_cloud", 1);
    pub_pcl_color = nh.advertise<sensor_msgs::PointCloud2>("/mechmind/textured_point_cloud", 1);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/mechmind/camera_info", 1);

    if (!findAndConnect(camera))
        throw mmind::eye::ErrorStatus{mmind::eye::ErrorStatus::MMIND_STATUS_INVALID_DEVICE,
                                      "Camera not found."};

    // Uncomment the following lines and comment the above if function to connect to a specific
    // camera by its IP address. The IP address is set in the "start_camera.launch" file as the
    // value of the "camera_ip" argument.

    // mmind::eye::ErrorStatus status;
    // mmind::eye::CameraInfo info;
    // info.firmwareVersion = mmind::eye::Version("2.3.4");
    // info.ipAddress = camera_ip;
    // info.port = 5577;
    // status = camera.connect(info);
    // if (!status.isOK())
    // {
    //     throw status;
    // }
    // std::cout << "Connected to the camera successfully." << std::endl;

    mmind::eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    if (use_external_intri) {
        intrinsics.texture.cameraMatrix.fx = fx;
        intrinsics.texture.cameraMatrix.fy = fy;
        intrinsics.texture.cameraMatrix.cx = u;
        intrinsics.texture.cameraMatrix.cy = v;
        intrinsics.depth.cameraMatrix.fx = fx;
        intrinsics.depth.cameraMatrix.fy = fy;
        intrinsics.depth.cameraMatrix.cx = u;
        intrinsics.depth.cameraMatrix.cy = v;
    } else {
        showError(camera.getCameraIntrinsics(intrinsics));
    }

    camera.setPointCloudUnit(mmind::eye::CoordinateUnit::Meter);
    capture_color_image_service = nh.advertiseService(
        "capture_color_image", &MechMindCamera::capture_color_image_callback, this);

    capture_stereo_color_images_service = nh.advertiseService(
        "capture_stereo_color_images", &MechMindCamera::capture_stereo_color_images_callback, this);

    capture_textured_point_cloud_service =
        nh.advertiseService("capture_textured_point_cloud",
                            &MechMindCamera::capture_textured_point_cloud_callback, this);

    capture_depth_map_service =
        nh.advertiseService("capture_depth_map", &MechMindCamera::capture_depth_map_callback, this);

    capture_point_cloud_service = nh.advertiseService(
        "capture_point_cloud", &MechMindCamera::capture_point_cloud_callback, this);

    add_user_set_service =
        nh.advertiseService("add_user_set", &MechMindCamera::add_user_set_callback, this);

    delete_user_set_service =
        nh.advertiseService("delete_user_set", &MechMindCamera::delete_user_set_callback, this);

    device_info_service =
        nh.advertiseService("device_info", &MechMindCamera::device_info_callback, this);

    get_all_user_sets_service =
        nh.advertiseService("get_all_user_sets", &MechMindCamera::get_all_user_sets_callback, this);

    get_current_user_set_service = nh.advertiseService(
        "get_current_user_set", &MechMindCamera::get_current_user_set_callback, this);

    save_all_settings_to_user_sets_service =
        nh.advertiseService("save_all_settings_to_user_sets",
                            &MechMindCamera::save_all_settings_to_user_sets_callback, this);

    set_current_user_set_service = nh.advertiseService(
        "set_current_user_set", &MechMindCamera::set_current_user_set_callback, this);

    set_int_parameter_service =
        nh.advertiseService("set_int_parameter", &MechMindCamera::set_int_parameter_callback, this);

    get_int_parameter_service =
        nh.advertiseService("get_int_parameter", &MechMindCamera::get_int_parameter_callback, this);

    set_float_parameter_service = nh.advertiseService(
        "set_float_parameter", &MechMindCamera::set_float_parameter_callback, this);

    get_float_parameter_service = nh.advertiseService(
        "get_float_parameter", &MechMindCamera::get_float_parameter_callback, this);

    set_bool_parameter_service = nh.advertiseService(
        "set_bool_parameter", &MechMindCamera::set_bool_parameter_callback, this);

    get_bool_parameter_service = nh.advertiseService(
        "get_bool_parameter", &MechMindCamera::get_bool_parameter_callback, this);

    set_enum_parameter_service = nh.advertiseService(
        "set_enum_parameter", &MechMindCamera::set_enum_parameter_callback, this);

    get_enum_parameter_service = nh.advertiseService(
        "get_enum_parameter", &MechMindCamera::get_enum_parameter_callback, this);

    set_range_parameter_service = nh.advertiseService(
        "set_range_parameter", &MechMindCamera::set_range_parameter_callback, this);

    get_range_parameter_service = nh.advertiseService(
        "get_range_parameter", &MechMindCamera::get_range_parameter_callback, this);

    set_roi_parameter_service =
        nh.advertiseService("set_roi_parameter", &MechMindCamera::set_roi_parameter_callback, this);

    get_roi_parameter_service =
        nh.advertiseService("get_roi_parameter", &MechMindCamera::get_roi_parameter_callback, this);

    set_float_sequence_parameter_service = nh.advertiseService(
        "set_float_array_parameter", &MechMindCamera::set_float_array_parameter_callback, this);

    get_float_sequence_parameter_service = nh.advertiseService(
        "get_float_array_parameter", &MechMindCamera::get_float_array_parameter_callback, this);
}

void MechMindCamera::publishColorCameraInfo(const std_msgs::Header& header, int width, int height)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";

    std::vector<double> distortionParams = {
        intrinsics.texture.cameraDistortion.k1, intrinsics.texture.cameraDistortion.k2,
        intrinsics.texture.cameraDistortion.p1, intrinsics.texture.cameraDistortion.p2,
        intrinsics.texture.cameraDistortion.k3};

    camera_info.D = distortionParams;

    std::vector<double> K{intrinsics.texture.cameraMatrix.fx,
                          0.0,
                          intrinsics.texture.cameraMatrix.cx,
                          0.0,
                          intrinsics.texture.cameraMatrix.fy,
                          intrinsics.texture.cameraMatrix.cy,
                          0.0,
                          0.0,
                          1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.K[i] = K[i];
    }

    std::vector<double> R{
        intrinsics.depthToTexture.rotation[0][0], intrinsics.depthToTexture.rotation[1][0],
        intrinsics.depthToTexture.rotation[2][0], intrinsics.depthToTexture.rotation[0][1],
        intrinsics.depthToTexture.rotation[1][1], intrinsics.depthToTexture.rotation[2][1],
        intrinsics.depthToTexture.rotation[0][2], intrinsics.depthToTexture.rotation[1][2],
        intrinsics.depthToTexture.rotation[2][2]};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.R[i] = R[i];
    }

    std::vector<double> P = {intrinsics.texture.cameraMatrix.fx,
                             0.0,
                             intrinsics.texture.cameraMatrix.cx,
                             -intrinsics.depthToTexture.translation[0],
                             0.0,
                             intrinsics.texture.cameraMatrix.fy,
                             intrinsics.texture.cameraMatrix.cy,
                             -intrinsics.depthToTexture.translation[1],
                             0.0,
                             0.0,
                             1.0,
                             0.0};
    for (size_t i = 0; i < 12; ++i) {
        camera_info.P[i] = P[i];
    }
    pub_camera_info.publish(camera_info);
}

void MechMindCamera::publishDepthCameraInfo(const std_msgs::Header& header, int width, int height)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";

    std::vector<double> distortionParams = {
        intrinsics.depth.cameraDistortion.k1, intrinsics.depth.cameraDistortion.k2,
        intrinsics.depth.cameraDistortion.p1, intrinsics.depth.cameraDistortion.p2,
        intrinsics.depth.cameraDistortion.k3};

    camera_info.D = distortionParams;

    std::vector<double> K{intrinsics.depth.cameraMatrix.fx,
                          0.0,
                          intrinsics.depth.cameraMatrix.cx,
                          0.0,
                          intrinsics.depth.cameraMatrix.fy,
                          intrinsics.depth.cameraMatrix.cy,
                          0.0,
                          0.0,
                          1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.K[i] = K[i];
    }

    std::vector<double> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.R[i] = R[i];
    }

    std::vector<double> P = {intrinsics.depth.cameraMatrix.fx,
                             0.0,
                             intrinsics.depth.cameraMatrix.cx,
                             0.0,
                             0.0,
                             intrinsics.depth.cameraMatrix.fy,
                             intrinsics.depth.cameraMatrix.cy,
                             0.0,
                             0.0,
                             0.0,
                             1.0,
                             0.0};
    for (size_t i = 0; i < 12; ++i) {
        camera_info.P[i] = P[i];
    }
    pub_camera_info.publish(camera_info);
}

void MechMindCamera::publishColorMap(mmind::eye::Color2DImage& color2DImage)
{
    cv::Mat color =
        cv::Mat(color2DImage.height(), color2DImage.width(), CV_8UC3, color2DImage.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_camera/color_map";
    ros_image.header.stamp = ros::Time::now();
    ;
    pub_color.publish(ros_image);
    publishColorCameraInfo(ros_image.header, color2DImage.width(), color2DImage.height());
    if (save_file) {
        const std::string path = "/tmp/image_2d.png";
        cv::imwrite(path, color);
        std::cout << "Capture and save the 2D image: " << path << std::endl;
    }
}

void MechMindCamera::publishStereoColorMap(mmind::eye::Color2DImage& leftColor2DImage,
                                           mmind::eye::Color2DImage& rightColor2DImage)
{
    cv::Mat colorLeft = cv::Mat(leftColor2DImage.height(), leftColor2DImage.width(), CV_8UC3,
                                leftColor2DImage.data());
    cv_bridge::CvImage cv_image_left;
    cv_image_left.image = colorLeft;
    cv_image_left.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::Image ros_image_left;
    cv_image_left.toImageMsg(ros_image_left);
    ros_image_left.header.frame_id = "mechmind_camera/left_color_map";
    ros_image_left.header.stamp = ros::Time::now();
    ;
    pub_color_left.publish(ros_image_left);
    publishDepthCameraInfo(ros_image_left.header, leftColor2DImage.width(),
                           leftColor2DImage.height());

    cv::Mat colorRight = cv::Mat(rightColor2DImage.height(), rightColor2DImage.width(), CV_8UC3,
                                 rightColor2DImage.data());
    cv_bridge::CvImage cv_image_right;
    cv_image_right.image = colorRight;
    cv_image_right.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::Image ros_image_right;
    cv_image_right.toImageMsg(ros_image_right);
    ros_image_right.header.frame_id = "mechmind_camera/right_color_map";
    ros_image_right.header.stamp = ros::Time::now();
    ;
    pub_color_right.publish(ros_image_right);
    publishColorCameraInfo(ros_image_right.header, rightColor2DImage.width(),
                           rightColor2DImage.height());

    if (save_file) {
        const std::string pathLeft = "/tmp/left_stereo.png";
        cv::imwrite(pathLeft, colorLeft);
        std::cout << "Capture and save the left stereo 2D image: " << pathLeft << std::endl;

        const std::string pathRight = "/tmp/right_stereo.png";
        cv::imwrite(pathRight, colorRight);
        std::cout << "Capture and save the right stereo 2D image: " << pathRight << std::endl;
    }
}

void MechMindCamera::publishDepthMap(mmind::eye::DepthMap& depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_camera/depth_map";
    ros_depth.header.stamp = ros::Time::now();
    ;
    pub_depth.publish(ros_depth);
    publishDepthCameraInfo(ros_depth.header, depthMap.width(), depthMap.height());
    if (save_file) {
        bool success = cv::imwrite("/tmp/depth_map.tiff", depth);
        if (success) {
            std::cout << "The depth map is saved to /tmp." << std::endl;
        } else {
            std::cerr << "Failed to save depth map." << std::endl;
        }
    }
}

void MechMindCamera::publishPointCloud(mmind::eye::PointCloud& pointCloud)
{
    sensor_msgs::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = "mechmind_camera/point_cloud";
    ros_cloud.header.stamp = ros::Time::now();
    ;
    convertToROSMsg(pointCloud, ros_cloud);
    pub_pcl.publish(ros_cloud);
    publishDepthCameraInfo(ros_cloud.header, pointCloud.width(), pointCloud.height());
}

void MechMindCamera::publishColorPointCloud(mmind::eye::TexturedPointCloud& texturedPointCloud)
{
    sensor_msgs::PointCloud2 ros_color_cloud;
    ros_color_cloud.header.frame_id = "mechmind_camera/textured_point_cloud";
    ros_color_cloud.header.stamp = ros::Time::now();
    ;
    convertToROSMsg(texturedPointCloud, ros_color_cloud);
    pub_pcl_color.publish(ros_color_cloud);
    publishDepthCameraInfo(ros_color_cloud.header, texturedPointCloud.width(),
                           texturedPointCloud.height());
}

bool MechMindCamera::capture_color_image_callback(
    mecheye_ros_interface::CaptureColorImage::Request& req,
    mecheye_ros_interface::CaptureColorImage::Response& res)
{
    mmind::eye::Frame2D frame;
    auto status = camera.capture2D(frame);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    auto colorMap = frame.getColorImage();
    publishColorMap(colorMap);
    return true;
}

bool MechMindCamera::capture_stereo_color_images_callback(
    mecheye_ros_interface::CaptureStereoColorImages::Request& req,
    mecheye_ros_interface::CaptureStereoColorImages::Response& res)
{
    mmind::eye::Frame2D frameLeft;
    mmind::eye::Frame2D frameRight;
    auto status = camera.captureStereo2D(frameLeft, frameRight);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    auto colorMapLeft = frameLeft.getColorImage();
    auto colorMapRight = frameRight.getColorImage();
    publishStereoColorMap(colorMapLeft, colorMapRight);
    return true;
}

bool MechMindCamera::capture_depth_map_callback(
    mecheye_ros_interface::CaptureDepthMap::Request& req,
    mecheye_ros_interface::CaptureDepthMap::Response& res)
{
    mmind::eye::Frame3D frame;
    auto status = camera.capture3D(frame);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    auto depthMap = frame.getDepthMap();
    publishDepthMap(depthMap);
    return true;
}

bool MechMindCamera::capture_point_cloud_callback(
    mecheye_ros_interface::CapturePointCloud::Request& req,
    mecheye_ros_interface::CapturePointCloud::Response& res)
{
    mmind::eye::Frame3D frame;
    auto status = camera.capture3D(frame);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    auto pointCloud = frame.getUntexturedPointCloud();
    publishPointCloud(pointCloud);
    if (save_file) {
        std::cout << "The point cloud is saved to /tmp." << std::endl;
        frame.saveUntexturedPointCloud(mmind::eye::FileFormat::PLY, "/tmp/point_cloud.ply");
    }
    return true;
}

bool MechMindCamera::capture_textured_point_cloud_callback(
    mecheye_ros_interface::CaptureTexturedPointCloud::Request& req,
    mecheye_ros_interface::CaptureTexturedPointCloud::Response& res)
{
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    mmind::eye::ErrorStatus status = camera.capture2DAnd3D(frame2DAnd3D);

    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    mmind::eye::TexturedPointCloud texturedPointCloud = frame2DAnd3D.getTexturedPointCloud();
    publishColorPointCloud(texturedPointCloud);
    if (save_file) {
        std::cout << "The textured point cloud is saved to /tmp." << std::endl;
        frame2DAnd3D.saveTexturedPointCloud(mmind::eye::FileFormat::PLY,
                                            "/tmp/textured_point_cloud.ply");
    }
    return true;
}

bool MechMindCamera::device_info_callback(mecheye_ros_interface::DeviceInfo::Request& req,
                                          mecheye_ros_interface::DeviceInfo::Response& res)
{
    mmind::eye::CameraInfo cameraInfo;
    mmind::eye::ErrorStatus status = camera.getCameraInfo(cameraInfo);
    showError(status);
    res.model = cameraInfo.model.c_str();
    res.serial_number = cameraInfo.serialNumber.c_str();
    res.hardware_version = cameraInfo.hardwareVersion.toString();
    res.firmware_version = cameraInfo.firmwareVersion.toString();
    res.ip_address = cameraInfo.ipAddress.c_str();
    res.subnet_mask = cameraInfo.subnetMask.c_str();
    res.ip_assignment_method = ipAssignmentMethodToString(cameraInfo.ipAssignmentMethod);
    res.port = cameraInfo.port;
    return true;
}

bool MechMindCamera::get_all_user_sets_callback(
    mecheye_ros_interface::GetAllUserSets::Request& req,
    mecheye_ros_interface::GetAllUserSets::Response& res)
{
    std::vector<std::string> sequence;
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.getAllUserSetNames(sequence);
    showError(status);
    res.sequence = sequence;
    return true;
}

bool MechMindCamera::get_current_user_set_callback(
    mecheye_ros_interface::GetCurrentUserSet::Request& req,
    mecheye_ros_interface::GetCurrentUserSet::Response& res)
{
    std::string value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getName(value);
    showError(status);
    res.value = value.c_str();
    return true;
}

bool MechMindCamera::set_current_user_set_callback(
    mecheye_ros_interface::SetCurrentUserSet::Request& req,
    mecheye_ros_interface::SetCurrentUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.selectUserSet(req.value.c_str());
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::add_user_set_callback(mecheye_ros_interface::AddUserSet::Request& req,
                                           mecheye_ros_interface::AddUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.addUserSet({req.value.c_str()});
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::delete_user_set_callback(mecheye_ros_interface::DeleteUserSet::Request& req,
                                              mecheye_ros_interface::DeleteUserSet::Response& res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.deleteUserSet({req.value.c_str()});
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::save_all_settings_to_user_sets_callback(
    mecheye_ros_interface::SaveAllSettingsToUserSets::Request& req,
    mecheye_ros_interface::SaveAllSettingsToUserSets::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.saveAllParametersToDevice();
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_float_array_parameter_callback(
    mecheye_ros_interface::GetFloatArrayParameter::Request& req,
    mecheye_ros_interface::GetFloatArrayParameter::Response& res)
{
    std::vector<double> array;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatArrayValue(req.name, array);
    showError(status);
    res.array = array;
    return true;
}

bool MechMindCamera::set_float_array_parameter_callback(
    mecheye_ros_interface::SetFloatArrayParameter::Request& req,
    mecheye_ros_interface::SetFloatArrayParameter::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatArrayValue(req.name, req.array);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_roi_parameter_callback(
    mecheye_ros_interface::GetROIParameter::Request& req,
    mecheye_ros_interface::GetROIParameter::Response& res)
{
    mmind::eye::ROI roi;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getRoiValue(req.name, roi);
    showError(status);
    res.x = roi.upperLeftX;
    res.y = roi.upperLeftY;
    res.width = roi.width;
    res.height = roi.height;
    return true;
}

bool MechMindCamera::set_roi_parameter_callback(
    mecheye_ros_interface::SetROIParameter::Request& req,
    mecheye_ros_interface::SetROIParameter::Response& res)
{
    mmind::eye::ROI roi{req.x, req.y, req.width, req.height};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setRoiValue(req.name, roi);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_range_parameter_callback(
    mecheye_ros_interface::GetRangeParameter::Request& req,
    mecheye_ros_interface::GetRangeParameter::Response& res)
{
    mmind::eye::Range<int> range{0, 0};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getRangeValue(req.name, range);
    showError(status);
    res.lower = range.min;
    res.upper = range.max;
    return true;
}

bool MechMindCamera::set_range_parameter_callback(
    mecheye_ros_interface::SetRangeParameter::Request& req,
    mecheye_ros_interface::SetRangeParameter::Response& res)
{
    mmind::eye::Range<int> range{req.lower, req.upper};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setRangeValue(req.name, range);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_int_parameter_callback(
    mecheye_ros_interface::GetIntParameter::Request& req,
    mecheye_ros_interface::GetIntParameter::Response& res)
{
    int value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getIntValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindCamera::set_int_parameter_callback(
    mecheye_ros_interface::SetIntParameter::Request& req,
    mecheye_ros_interface::SetIntParameter::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setIntValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_bool_parameter_callback(
    mecheye_ros_interface::GetBoolParameter::Request& req,
    mecheye_ros_interface::GetBoolParameter::Response& res)
{
    bool value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getBoolValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindCamera::set_bool_parameter_callback(
    mecheye_ros_interface::SetBoolParameter::Request& req,
    mecheye_ros_interface::SetBoolParameter::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setBoolValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_enum_parameter_callback(
    mecheye_ros_interface::GetEnumParameter::Request& req,
    mecheye_ros_interface::GetEnumParameter::Response& res)
{
    std::string value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getEnumValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindCamera::set_enum_parameter_callback(
    mecheye_ros_interface::SetEnumParameter::Request& req,
    mecheye_ros_interface::SetEnumParameter::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setEnumValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}

bool MechMindCamera::get_float_parameter_callback(
    mecheye_ros_interface::GetFloatParameter::Request& req,
    mecheye_ros_interface::GetFloatParameter::Response& res)
{
    double value = -1;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatValue(req.name, value);
    showError(status);
    res.value = value;
    return true;
}

bool MechMindCamera::set_float_parameter_callback(
    mecheye_ros_interface::SetFloatParameter::Request& req,
    mecheye_ros_interface::SetFloatParameter::Response& res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatValue(req.name, req.value);
    showError(status);
    res.error_code = status.errorCode;
    res.error_description = status.errorDescription.c_str();
    return true;
}
