cmake_minimum_required(VERSION 3.5)
project (mecheye_profiler_ros_interface)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARDREQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE on)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp rospy std_msgs genmsg sensor_msgs tf message_generation)

add_service_files(
  DIRECTORY
  srv
  FILES
  SetCurrentUserSet.srv
  SaveAllSettingsToUserSets.srv
  AddUserSet.srv
  StartAcquisition.srv
  StopAcquisition.srv
  TriggerSoftware.srv
  DeleteUserSet.srv
  ProfilerInfo.srv
  GetAllUserSets.srv
  GetCurrentUserSet.srv
  SetIntParameter.srv
  GetIntParameter.srv
  SetBoolParameter.srv
  GetBoolParameter.srv
  SetEnumParameter.srv
  GetEnumParameter.srv
  SetFloatParameter.srv
  GetFloatParameter.srv
  SetProfileROIParameter.srv
  GetProfileROIParameter.srv
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
add_dependencies(start mecheye_profiler_ros_interface_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(start
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${catkin_LIBRARIES}
    MechEyeApi
    )
