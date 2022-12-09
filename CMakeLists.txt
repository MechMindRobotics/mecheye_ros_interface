cmake_minimum_required(VERSION 3.5)
project (mecheye_ros_interface)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARDREQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/jsoncpp")
set(CMAKE_VERBOSE_MAKEFILE on)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp rospy std_msgs genmsg sensor_msgs tf message_generation)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_service_files(
  DIRECTORY
  srv
  FILES
  AddUserSet.srv
  CaptureColorMap.srv
  CaptureColorPointCloud.srv
  CaptureDepthMap.srv
  CapturePointCloud.srv
  DeleteUserSet.srv
  DeviceInfo.srv
  Get2DExpectedGrayValue.srv
  Get2DExposureMode.srv
  Get2DExposureSequence.srv
  Get2DExposureTime.srv
  Get2DROI.srv
  Get2DSharpenFactor.srv
  Get2DToneMappingEnable.srv
  Get3DExposure.srv
  Get3DGain.srv
  Get3DROI.srv
  GetAllUserSets.srv
  GetCloudOutlierFilterMode.srv
  GetCloudSmoothMode.srv
  GetCurrentUserSet.srv
  GetDepthRange.srv
  GetFringeContrastThreshold.srv
  GetFringeMinThreshold.srv
  GetLaserSettings.srv
  GetUhpCaptureMode.srv
  GetUhpFringeCodingMode.srv
  GetUhpSettings.srv
  
  GetProjectorFringeCodingMode.srv
  GetProjectorPowerLevel.srv
  GetProjectorAntiFlickerMode.srv
  
  SaveAllSettingsToUserSets.srv
  Set2DExpectedGrayValue.srv
  Set2DExposureMode.srv
  Set2DExposureSequence.srv
  Set2DExposureTime.srv
  Set2DROI.srv
  Set2DSharpenFactor.srv
  Set2DToneMappingEnable.srv
  Set3DExposure.srv
  Set3DGain.srv
  Set3DROI.srv
  SetCloudOutlierFilterMode.srv
  SetCloudSmoothMode.srv
  SetCurrentUserSet.srv
  SetDepthRange.srv
  SetFringeContrastThreshold.srv
  SetFringeMinThreshold.srv
  SetLaserSettings.srv
  SetUhpCaptureMode.srv
  SetUhpFringeCodingMode.srv
  SetUhpSettings.srv
  
  SetProjectorFringeCodingMode.srv
  SetProjectorPowerLevel.srv
  SetProjectorAntiFlickerMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs roscpp message_runtime)


include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    /opt/mech-mind/mech-eye-sdk/include
    )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

link_directories(/opt/mech-mind/mech-eye-sdk/lib)

aux_source_directory(src SRC_LIST)
add_executable(start ${SRC_LIST})
add_dependencies(start mecheye_ros_interface_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(start
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    MechEyeApi
    )
