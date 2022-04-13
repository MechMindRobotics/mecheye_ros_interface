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
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp rospy std_msgs genmsg sensor_msgs tf)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs roscpp)


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
