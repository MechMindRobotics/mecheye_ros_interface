# ROS 1 Interface for Mech-Eye Industrial 3D Profiler

This documentation provides instructions on using the ROS 1 interface for Mech-Eye 3D Laser Profiler.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Prerequisites

In order to use the ROS 1 interface, the following prerequisites must be satisfied:

* Ubuntu version: 20.04 is recommended, which is the main target platform of ROS Noetic Ninjemys.
* ROS 1 version: The stable version [Noetic Ninjemys](http://roswiki.autolabor.com.cn/noetic%282f%29Installation%282f%29Ubuntu.html) is recommended.

  >Note: The ROS 1 interface of Mech-Eye SDK has been tested with the above versions of ROS and Ubuntu. The command examples in this document are based on the above versions.

* [Download the latest version of Mech-Eye SDK](https://downloads.mech-mind.com/?tab=tab-sdk).
* Dependencies:

  * OpenCV: 3.0 or above
  * PCL: 1.8 or above

### Install Mech-Eye SDK

>Note: If you have installed Mech-Eye SDK before, please uninstall it first with the following command:
>
>```bash
>sudo dpkg -P MechEyeApi
>```

* If the system architecture is AMD64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_amd64.deb'
  ```

* If the system architecture is ARM64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_arm64.deb'
  ```

### Install Dependencies

After ROS Noetic Ninjemys has been installed, execute the following commands to install the dependencies.

```bash
sudo apt install libopencv-dev
sudo apt install ros-noetic-cv-bridge
sudo apt install libpcl-dev
sudo apt install ros-noetic-pcl-conversions
sudo apt install python3-colcon-common-extensions
```

## Clone and Build the Interface

1. Execute the following commands to clone the interface:

   ```bash
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
   git clone https://github.com/MechMindRobotics/mecheye_profiler_ros_interface.git
   ```

2. Execute the following command to set up your environment:

   ```bash
   source /opt/ros/noetic/setup.bash
   ```

3. Execute the following commands to build the interface:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## Use the Interface

1. (Optional) Change the configurations in `~/catkin_ws/src/mecheye_profiler_ros_interface/launch/start_profiler.launch` according to your needs:

   * `save_file`: Set this argument to `true` to allow file saving to the `/tmp/` directory. If you set this argument to `false`, the obtained data are not saved locally automatically.
   * `profiler_ip`: If you want to connect to a specific profiler by its IP address, change the value of this argument to the IP address of your profiler. You also need to the `if (!findAndConnect(profiler))` function and uncomment the lines below this function in `~/catkin_ws/src/mecheye_profiler_ros_interface/src/MechMindProfiler.cpp`.

   > Note: Remember to run `catkin_make` again after making changes to the `MechMindProfiler.h` and `*.cpp`.

2. Open a terminal and execute the following command to start up the interface:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch mecheye_profiler_ros_interface start_profiler.launch
   ```

3. Enter the index number of the profiler to which you want to connect, and press the Enter key.
4. Open a new terminal, and execute the following command to invoke a service. Replace `service_name` with the name of the service, `parameter_name` with the name of the profiler parameter, and `parameter_value` with the parameter value to be set.

   >Note: For example commands of each service, refer to the [Services](#services) section.

   ```bash
   source ~/catkin_ws/devel/setup.bash
   rosservice call [/service_name] "{parameter_name: parameter_value}"
   ```

## Topics

The following topics are provided:

* /mechmind_profiler/intensity_image: the 2D image encoded as "mono8"
* /mechmind_profiler/depth_map: the depth map encoded as a single-channel image, the channel containing a 32-bit float number
* /mechmind_profiler/point_cloud: the untextured point cloud data
* /mechmind_profiler/textured_point_cloud: the textured point cloud data

## Services

### Acquire Data

#### [start_acquisition](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/StartAcquisition.srv)

Invoke this service to enter the laser profiler into acquisition ready status, where it can accept trigger signals for scanning.

Example:

```bash
rosservice call /start_acquisition
```

#### [stop_acquisition](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/StopAcquisition.srv)

Invoke this service to exit the laser profiler from acquisition ready status to avoid accidental triggering of scanning.

Example:

```bash
rosservice call /stop_acquisition
```

#### [trigger_software](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/TriggerSoftware.srv)

Invoke this service to send a software signal to trigger data acquisition.

Example:

```bash
rosservice call /trigger_software
```

### Manage Parameter Groups

#### [get_all_user_sets](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetAllUserSets.srv)

Invoke this service to obtain the names of all available parameter groups.

Example:

  ```bash
  rosservice call /get_all_user_sets
  ```

#### [get_current_user_set](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetCurrentUserSet.srv)

Invoke this service to obtain the name of the currently selected parameter group.

Example:

  ```bash
  rosservice call /get_current_user_set
  ```

#### [set_current_user_set](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetCurrentUserSet.srv)

Invoke this service to select the parameter group to be used.

This service has one parameter:

* `value` (string): the name of the parameter group to be selected.

Example: Select the "123" parameter group.

  ```bash
  rosservice call /set_current_user_set "{value: '123'}"
  ```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

#### [add_user_set](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/AddUserSet.srv)

Invoke this service to add a parameter group. The newly added parameter group is automatically selected as the current parameter group.

This service has one parameter:

* `value` (string): the name of the parameter group to be added.

Example: Add a parameter group named "123".

```bash
rosservice call /add_user_set "{value: '123'}"
```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

#### [delete_user_set](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/DeleteUserSet.srv)

Invoke this service to delete the specified parameter group.

This service has one parameter:

* `value` (string): the name of the parameter group to be deleted.

Example: Delete the parameter group named "123".

  ```bash
  rosservice call /delete_user_set "{value: '123'}"
  ```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

### Obtain Profiler Information

#### [profiler_info](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/ProfilerInfo.srv)

Invoke this service to print the following information of the currently connected profiler:

* Model
* Controller serial number
* Sensor serial number
* Hardware version
* Firmware version
* IP address
* Subnet mask
* IP address assignment method
* Port

Example:

  ```bash
  rosservice call /device_info
  ```

### Adjust Profiler Parameters

> Note: Mech-Eye SDK 2.3.4 and above provide methods according to the data types of the profiler parameters, and the ROS 1 interface provides the corresponding services. To obtain or adjust the value of a profiler parameter, call the service corresponding to the data type of the profiler parameter and input the name of the profiler parameter as the service's parameter. The data types and names of the profiler parameters can be found in the header files in the installation path of Mech-Eye SDK: `/opt/mech-mind/mech-eye-sdk/include/profiler/parameters/`.

The following data types of profiler parameters are distinguished:

* _Int
* _Float
* _Bool
* _Enum
* _ ProfileRoi

#### [get_int_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetIntParameter.srv)

Invoke this service to obtain the value of the specified _Int-type profiler parameter.

This service has one parameter:

* `name` (string): the name of the profiler parameter.

Example: Obtain the value of the `ScanLineCount` parameter.

  ```bash
  rosservice call /get_int_parameter "{name: ScanLineCount}"
  ```

### [set_int_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetIntParameter.srv)

Invoke this service to set the value of the specified _Int-type profiler parameter.

This service has two parameters:

* `name` (string): the name of the profiler parameter.
* `value` (int): the new value of the profiler parameter.

Example: Set the value of the `ScanLineCount` parameter to 2000.

  ```bash
  rosservice call /set_int_parameter "{name: ScanLineCount, value: 2000}"
  ```

### [get_float_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetFloatParameter.srv)

Invoke this service to obtain the value of the specified _Float-type profiler parameter.

This service has one parameter:

* `name` (string): the name of the profiler parameter.

Example: Obtain the value of the `SoftwareTriggerRate` parameter.

  ```bash
  rosservice call /get_float_parameter "{name: SoftwareTriggerRate}"
  ```

### [set_float_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetFloatParameter.srv)

Invoke this service to set the value of the specified _Float-type profiler parameter.

This service has two parameters:

* `name` (string): the name of the profiler parameter.
* `value` (float): the new value of the profiler parameter.

Example: Set the value of the `SoftwareTriggerRate` parameter to 3000.

  ```bash
  rosservice call /set_float_parameter "{name: SoftwareTriggerRate, value: 3000}"
  ```

### [get_bool_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetBoolParameter.srv)

Invoke this service to obtain the value of the specified _Bool-type profiler parameter.

This service has one parameter:

* `name` (string): the name of the profiler parameter.

Example: Obtain the value of the `EnableZAxisAlignment` parameter.

  ```bash
  rosservice call /get_bool_parameter "{name: EnableZAxisAlignment}"
  ```

### [set_bool_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetBoolParameter.srv)

Invoke this service to set the value of the specified _Bool-type profiler parameter.

This service has two parameters:

* `name` (string): the name of the profiler parameter.
* `value` (bool): the new value of the profiler parameter.

Example: Set the value of the `EnableZAxisAlignment` parameter to `True`.

  ```bash
  rosservice call /set_bool_parameter "{name: EnableZAxisAlignment, value: True}"
  ```

### [get_enum_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetEnumParameter.srv)

Invoke this service to obtain the value of the specified _Enum-type profiler parameter.

This service has one parameter:

* `name` (string): the name of the profiler parameter.

Example: Obtain the value of the `DataAcquisitionTriggerSource` parameter.

  ```bash
  rosservice call /get_enum_parameter "{name: DataAcquisitionTriggerSource}"
  ```

### [set_enum_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetEnumParameter.srv)

Invoke this service to set the value of the specified _Enum-type profiler parameter.

This service has two parameters:

* `name` (string): the name of the profiler parameter.
* `value` (string): the new value of the profiler parameter.

Example: Set the value of the `DataAcquisitionTriggerSource` parameter to `Software`.

  ```bash
  rosservice call /set_enum_parameter "{name: DataAcquisitionTriggerSource, value: Software}"
  ```

### [get_profile_roi_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/GetProfileROIParameter.srv)

Invoke this service to obtain the value of the specified _ProfileRoi-type profiler parameter.

This service has one parameter:

* `name` (string): the name of the profiler parameter.

Example: Obtain the value of the `ROI` parameter.

  ```bash
  rosservice call /get_profile_roi_parameter "{name: ROI}"
  ```

### [set_profile_roi_parameter](https://github.com/MechMindRobotics/mecheye_profiler_ros_interface/blob/master/srv/SetProfileROIParameter.srv)

Invoke this service to set the value of the specified _ProfileRoi-type profiler parameter.

This service has five parameters:

* `name` (string): the name of the profiler parameter.
* `x_axis_center` (float64): the new center's x-coordinate of the ROI.
* `width` (float64): the new width of the ROI.
* `height` (float64): the new height of the ROI.

Example: Set the value of the `ROI` parameter to [30, 10, 30] (which is an ROI that is 10 mm wide, 40 mm high and has its center's x-coordinate is 30mm).

  ```bash
  rosservice call /set_roi_parameter "{name: ROI, x_axis_center: 30, width: 10, height: 30}"
  ```
