# Mech-Eye ROS Interface

This repository contains the ROS 1 interface for Mech-Eye Industrial 3D Camera.

## Installation

### ROS and Ubuntu Version Requirements

- On Ubuntu 18.04: use with [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- On Ubuntu 20.04: use with [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Dependencies

|   Package    |    Version    |
| :----------: | :-----------: |
|    OpenCV    |     >= 3      |
|     PCL      |     >= 1.8    |
|    Eigen     |     3.3.0     |
|     VTK      |     6.3.0     |
| Mech-Eye API |     latest    |

### Install Mech-Eye API

Download Mech-Eye API from [Mech-Mind's website](https://www.mech-mind.com/download/softwaredownloading.html) and install it with `sudo apt`.

### Clone and Compile the Interface

Clone and compile the interface with the following commands:

  ```bash
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface
  cd ~/catkin_ws
  catkin_make
  ```

## Start Using the Interface

1. Source the build workspace and launch the interface:

    ```bash
    source ~/catkin_ws/devel/setup.bash
    roslaunch mecheye_ros_interface start_camera.launch
    ```
2. Enter the index number of the camera to which you want to connect, and press the Enter key. 
3. Open a new terminal, source the workspace and invoke a service：

    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosservice call [/service] [arguments]
    ```
    > Note: For code examples of each service, check the "Services" section below.

- Interface functions are described in the online documentation [Mech-Eye API Reference](https://docs.mech-mind.net/latest/en-GB/MechEye/MechEyeAPI/ApiReference/ApiReference.html).

- Change the following configurations in `~/catkin_ws/src/mecheye_ros_interface/launch/start_camera.launch` according to your needs:
  - `save_file`: To enable file saving to the designated path, change the default value to `true`.
    > Note: You can change the paths for file saving in `/mecheye_ros_interface/src/MechMindCamera.cpp`.
  - `camera_ip`: Change the default value to the IP address of your camera.
    > Note: To connect to a camera by its IP address, you also need to edit `MechMindCamera.cpp`. Uncomment lines 79 to 90, and comment lines 74 to 75 and lines 92 to 94.
  <!-- - `tf_[]` arguments: If you need to convert the camera reference frame to another frame, change these arguments to the transformation parameters. Use quaternions for rotation. -->
  > Note: Remember to run `catkin_make` again after making changes to `start_camera.launch` and `*.cpp`.

## Topics

### /mechmind/camera_info

Camera calibration and metadata.

### /mechmind/color_image

Color image encoded as "bgr8."

### /mechmind/depth_image

Depth image encoded as 32-bit float.

### /mechmind/point_cloud

Point cloud data.

### /mechmind/color_point_cloud

Textured point cloud data.

## Services

### [add_user_set](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/AddUserSet.srv)

Invoke this service to add a parameter group. The newly added parameter group is automatically selected as the current parameter group.

This service has one parameter:

- `value` (string): the name of the parameter group to be added.

Example: add a parameter group named "123"
  
  ```bash
  rosservice call /add_user_set '!!str 123' # For parameter group names that consist of numbers only, add "!!str" before the name.
  ```

### [capture_color_map](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/CaptureColorMap.srv)

Invoke this service to obtain a 2D image.

Example: 

  ```bash
  rosservice call /capture_color_map
  ```

### [capture_color_point_cloud](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/CaptureColorPointCloud.srv)

Invoke this service to obtain a textured point cloud.

Example:

  ```bash
  rosservice call /capture_color_point_cloud
  ```

### [capture_depth_map](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/CaptureDepthMap.srv)

Invoke this service to obtain a depth map.

Example: 

  ```bash
  rosservice call /capture_depth_map
  ```

### [capture_point_cloud](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/CapturePointCloud.srv)

Invoke this service to obtain an untextured point cloud.

Example: 

  ```bash
  rosservice call /capture_point_cloud
  ```

### [delete_user_set](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/DeleteUserSet.srv)

Invoke this service to delete the specified parameter group.

This service has one parameter:

- `value` (string): the name of the parameter group to be deleted.

Example: delete the parameter group named "123"

  ```bash
  rosservice call /delete_user_set '!!str 123' # For parameter group names that consist of numbers only, add "!!str" before the name.
  ```


### [device_info](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/DeviceInfo.srv)

Invoke this service to print the following information of the currently connected camera:

- Model
- Serial number (ID)
- Hardware version
- Firmware version
- IP address
- Port

Example: 

  ```bash
  rosservice call /device_info
  ```

### [get_2d_expected_gray_value](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DExpectedGrayValue.srv)

Invoke this service to obtain the current `ExpectedGrayValue` value.

Example: 

  ```bash
  rosservice call /get_2d_expected_gray_value
  ```

### [get_2d_exposure_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DExposureMode.srv)

Invoke this service to obtain the current `Scan2DExposureMode` value.

Example: 

  ```bash
  rosservice call /get_2d_exposure_mode
  ```

### [get_2d_exposure_sequence](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DExposureSequence.srv)

Invoke this service to obtain the current `HDRExposureSequence` value.

Example: 

  ```bash
  rosservice call /get_2d_exposure_sequence
  ```

### [get_2d_exposure_time](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DExposureTime.srv)

Invoke this service to obtain the current 2D `ExposureTime` value.

Example: 

  ```bash
  rosservice call /get_2d_exposure_time
  ```

### [get_2d_roi](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DROI.srv)

Invoke this service to obtain the current `Scan2DROI` values.

Example: 

  ```bash
  rosservice call /get_2d_roi
  ```

### [get_2d_sharpen_factor](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DSharpenFactor.srv)

Invoke this service to obtain the current `SharpenFactor` value.

Example: 

  ```bash
  rosservice call /get_2d_sharpen_factor
  ```

### [get_2d_tone_mapping](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get2DToneMappingEnable.srv)

Invoke this service to obtain the current `ToneMappingEnable` value.

Example: 

  ```bash
  rosservice call /get_2d_tone_mapping
  ```

### [get_3d_exposure](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get3DExposure.srv)

Invoke this service to get the current 3D `ExposureSequence` values.

Example: 

  ```bash
  rosservice call /get_3d_exposure
  ```

### [get_3d_gain](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get3DGain.srv)

Invoke this service to obtain the current `Gain` value.

Example: 

  ```bash
  rosservice call /get_3d_gain
  ```

### [get_3d_roi](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Get3DROI.srv)

Invoke this service to obtain the current `Scan3DROI` values.

Example: 

  ```bash
  rosservice call /get_3d_roi
  ```

### [get_all_user_sets](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetAllUserSets.srv)

Invoke this service to obtain the names of all available parameter groups.

Example: 

  ```bash
  rosservice call /get_all_user_sets
  ```


### [get_cloud_outlier_filter_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetCloudOutlierFilterMode.srv)

Invoke this service to get the current `CloudOutlierFilterMode` value.

Example: 

  ```bash
  rosservice call /get_cloud_outlier_filter_mode
  ```

### [get_cloud_smooth_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetCloudSmoothMode.srv)

Invoke this service to get the current `CloudSmoothMode` value.

Example: 

  ```bash
  rosservice call /get_cloud_smooth_mode
  ```

### [get_current_user_set](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetCurrentUserSet.srv)

Invoke this service to obtain the name of the currently selected parameter group.

Example: 

  ```bash
  rosservice call /get_current_user_set
  ```

### [get_depth_range](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetDepthRange.srv)

Invoke this service to obtain the current `DepthRange` value.

Example: 

  ```bash
  rosservice call /get_depth_range
  ```

### [get_fringe_contrast_threshold](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetFringeContrastThreshold.srv)

Invoke this service to obtain the current `FringeContrastThreshold` value.

Example: 

  ```bash
  rosservice call /get_fringe_contrast_threshold
  ```

### [get_fringe_min_threshold](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetFringeMinThreshold.srv)

Invoke this service to obtain the current `FringeMinThreshold` value.

Example: 

  ```bash
  rosservice call /get_fringe_min_threshold
  ```

### [get_laser_settings](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetLaserSettings.srv)

Invoke this service to obtain the current laser camera settings. 

> Note: This service is only available for a laser camera.

Example: 

  ```bash
  rosservice call /get_laser_settings
  ```

### [get_projector_antiflickermode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetProjectorAntiFlickerMode.srv)

Invoke this service to obtain the current `AntiFlickerMode` value. 

> Note: This service is only available for PRO (V4), PRO M (V4), NANO (V4), and Nano (V3).

Example: 

  ```bash
  rosservice call /get_projector_antiflickermode
  ```

### [get_projector_fringecodingmode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetProjectorFringeCodingMode.srv)

Invoke this service to obtain the current `FringeCodingMode` value. 

> Note: This service is only available for PRO S (V4), PRO M (V4), Nano (V3), and Pro XS (V3).

Example: 

  ```bash
  rosservice call /get_projector_fringecodingmode
  ```

### [get_projector_powerlevel](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/GetProjectorPowerLevel.srv)


Invoke this service to obtain the current `powerLevel` value. 

> Note: This service is only available for DLP cameras, excluding Deep (V3) and Pro L Enhanced (V3).

Example: 

  ```bash
  rosservice call /get_projector_powerlevel
  ```

### [get_uhp_settings](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetUhpSettings.srv)

Invoke this service to obtain the current UHP camera settings.

> Note: This service is only available for the UHP series.

Example: 

  ```bash
  rosservice call /get_uhp_settings
  ```

### [get_uhp_capture_mode](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetUhpCaptureMode.srv)

Invoke this service to obtain the current `UhpCaptureMode` value.

> Note: This service is only available for the UHP series.

Example: 

  ```bash
  rosservice call /get_uhp_capture_mode
  ```

### [get_uhp_fringe_coding_mode](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetUhpFringeCodingMode.srv)

Invoke this service to obtain the current `UhpFringeCodingMode` value.

> Note: This service is only available for the UHP series.

Example: 

  ```bash
  rosservice call /get_uhp_fringe_coding_mode
  ```

### [set_2d_expected_gray_value](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DExpectedGrayValue.srv)

Invoke this service to set the value of `ExpectedGrayValue` value.

> Note: `ExpectedGrayValue` only takes effect when `Scan2DExposureMode` is set to `Auto`.

This service has one parameter:

- `value` (int32): the `ExpectedGrayValue` value to be set. The value range is 0–255.

Example: set the value of `ExpectedGrayValue` to 20

  ```bash
  rosservice call /set_2d_expected_gray_value 20
  ```

### [set_2d_exposure_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DExposureMode.srv)

Invoke this service to set the value of `Scan2DExposureMode`.

This service has one parameter:

- `value` (string): the `Scan2DExposureMode` value to be set. Values include "Timed", "Auto", "HDR", and "Flash".

Example: set the value of `Scan2DExposureMode` to "HDR"

  ```bash
  rosservice call /set_2d_exposure_mode 'HDR'
  ```

### [set_2d_exposure_sequence](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DExposureSequence.srv)

Invoke this service to set the value of `HDRExposureSequence`.

> Note: `HDRExposureSequence` only takes effect when `Scan2DExposureMode` is set to `HDR`.

This service has one parameter:

- `sequence` (float64[]): the `HDRExposureSequence` values to be set.
  - Value range: 0.1–999.0
  - Number of elements in the sequence: 1–5

Example: set the values of `HDRExposureSequence` to 30.0, 35.5, and 40.0

  ```bash
  rosservice call /set_2d_exposure_sequence [30.0,35.5,40.0]# Do not put spaces between elements in the sequence. Otherwise the element after a space will be passed as a string.
  ```

### [set_2d_exposure_time](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DExposureTime.srv)

Invoke this service to set the value of 2D `ExposureTime`.

> Note: 2D `ExposureTime` only takes effect when `Scan2DExposureMode` is set to `Timed`.

This service has one parameter:

- `value` (float64): the 2D `ExposureTime` value to be set. The value range is 0.1–999.0.

Example: set the values of 2D `ExposureTime` to 35.5

  ```bash
  rosservice call /set_2d_exposure_time 35.5
  ```

### [set_2d_roi](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DROI.srv)

Invoke this service to obtain the values of `Scan2DROI`.

> Note: `Scan2DROI` only takes effect when `Scan2DExposureMode` is set to `Auto`.

This service has four parameters:

- `x` (uint32): the x coordinate of the upper-left corner of the auto-exposure ROI.
- `y` (uint32): the y coordinate of the upper-left corner of the auto-exposure ROI.
- `width` (uint32): the width of the auto-exposure ROI.
- `height` (uint32): the height of the auto-exposure ROI.

Example: set the values of 2D `Scan2DROI` to 20, 20 , 600, and 800

  ```bash
  rosservice call /set_2d_roi 20 20 600 800
  ```

### [set_2d_sharpen_factor](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DSharpenFactor.srv)

Invoke this service to set the value of `SharpenFactor`.

This service has one parameter:

- `value` (float64): the `SharpenFactor` value to be set. The value range is 0.0–5.0.

Example: set the value of `SharpenFactor` to 0.5

  ```bash
  rosservice call /set_2d_sharpen_factor 0.5
  ```

### [set_2d_tone_mapping](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set2DToneMappingEnable.srv)

Invoke this service to set the value of `ToneMappingEnable`.

This service has one parameter:

- `value` (bool): the `ToneMappingEnable` value to be set.

Example: set the value of `ToneMappingEnable` to True

  ```bash
  rosservice call /set_2d_tone_mapping True
  ```

### [set_3d_exposure](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set3DExposure.srv)

Invoke this service to set the value of 3D `ExposureSequence`.

This service has one parameter:

- `sequence` (float64[]): the 3D `ExposureSequence` values to be set. 
  - Value range: 0.1–99.0
  - Number of elements in the sequence: 1–3

Example: set the values of 3D `ExposureSequence` to 30.0, 35.5, and 40.0

  ```bash
  rosservice call /set_3d_exposure [30.0,35.5,40.0]# Do not put spaces between elements in the sequence. Otherwise the element after a space will be passed as a string.
  ```

### [set_3d_gain](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set3DGain.srv)

Invoke this service to set the value of `Gain`.

This service has one parameter:

- `value` (float64): the `Gain` value to be set. The value range is 0.0–16.0.

Example: set the value of `Gain` to 2.5

  ```bash
  rosservice call /set_3d_gain 2.5
  ```

### [set_3d_roi](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/Set3DROI.srv)

Invoke this service to set the value of `Scan3DROI`.

This service has four parameters:

- `x` (uint32): the x coordinate of the upper-left corner of the ROI.
- `y` (uint32): the y coordinate of the upper-left corner of the ROI.
- `width` (uint32): the width of the ROI.
- `height` (uint32): the height of the ROI.

Example: set the values of `Scan3DROI` to 20, 20, 600, and 800

  ```bash
  rosservice call /set_3d_roi 20 20 600 800
  ```

### [set_cloud_outlier_filter_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetCloudOutlierFilterMode.srv)

Invoke this service to set the value of `CloudOutlierFilterMode`.

This service has one parameter:

- `value` (string): the `CloudOutlierFilterMode` value to be set. Values include "Off", "Weak", and "Normal".


Example: set the value of `CloudOutlierFilterMode` to "Off"

  ```bash
  rosservice call /set_cloud_outlier_filter_mode '!!str Off' # The value "Off" must be input as '!!str Off'.
  ```

### [set_cloud_smooth_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetCloudSmoothMode.srv)

Invoke this service to set the value of `CloudSmoothMode`.

This service has one parameter:

- `value` (string): the `CloudSmoothMode` value to be set. Values include "Off", "Weak", "Normal" and "Strong".

Example: set the value of `CloudSmoothMode` to "Off "

  ```bash
  rosservice call /set_cloud_smooth_mode '!!str Off' # The value "Off" must be input as '!!str Off'.
  ```

### [set_current_user_set](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetCurrentUserSet.srv)

Invoke this service to select the parameter group to use.

This service has one parameter:

- `value` (string): the name of the parameter group to be selected. The "default" and "calib" parameter groups are built-in.

Example: select the "123" parameter group

  ```bash
  rosservice call /set_current_user_set '!!str 123' # For parameter group names that consist of numbers only, add "!!str" before the name.
  ```

### [set_depth_range](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetDepthRange.srv)

Invoke this service to set the value of `DepthRange`.

This service has two parameters:

- `lower` (int32): The lower limit of the depth range. The value range is 1–4000.
- `upper` (int32): The upper limit of the depth range. The value range is 2-5000 (must be greater than the value of `lower`).

Example: set the values of `DepthRange` to 300 and 1000

  ```bash
  rosservice call /set_depth_range 300 1000
  ```

### [set_fringe_contrast_threshold](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetFringeContrastThreshold.srv)

Invoke this service to set the value of `FringeContrastThreshold`.

This service has one parameter:

- `value` (int32): the `FringeContrastThreshold` value to be set. The value range is 1-100.

Example: set the values of `FringeContrastThreshold` to 3

  ```bash
  rosservice call /set_fringe_contrast_threshold 3
  ```

### [set_fringe_min_threshold](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetFringeMinThreshold.srv)

Invoke this service to set the value of `FringeMinThreshold`.

This service has one parameter:

- `value` (int32): the `FringeMinThreshold` value to be set. The value range is 1-100.

Example: set the values of `FringeMinThreshold` to 3

  ```bash
  rosservice call /set_fringe_min_threshold 3
  ```

### [set_laser_settings](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetLaserSettings.srv)

Invoke this service to set the laser camera settings.

> Note: This service is only available for a laser camera.

This service has five parameters:

- `fringeCodingMode` (string): the `LaserFringeCodingMode` value to be set. Values include "Fast" and "Accurate".
- `frameRangeStart` (int32): the `frameRangeStart` value to be set. The value range is 0-100.
- `frameRangeEnd` (int32): the `frameRangeEnd` value to be set. The value range is 0-100 (also must satisfy: `frameRangeEnd` - `frameRangeStart` >= 25).
- `framePartitionCount` (int32): the `framePartitionCount` value to be set. The value range is 1-4.
- `powerLevel` (int32): the `powerLevel` value to be set. The value range is 20-100.

Example: set the values of the laser camera settings to "Fast", 20, 80, 2, and 60.

  ```bash
  rosservice call /set_laser_settings 'Fast' 20 80 2 60
  ```

### [set_projector_antiflickermode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetProjectorAntiFlickerMode.srv)

Invoke this service to set the value of `AntiFlickerMode`.

> Note: This service is only available for PRO (V4), PRO M (V4), NANO (V4), and Nano (V3).

This service has one parameter:

- `value` (string): the `AntiFlickerMode` value to be set. Values include "Off", "AC50Hz", and "AC60Hz".

Example: set the value of `AntiFlickerMode` to "Off"

  ```bash
  rosservice call /set_projector_antiflickermode '!!str Off'# The value "Off" must be input as '!!str Off'.
  ```

### [set_projector_fringecodingmode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetProjectorFringeCodingMode.srv)

Invoke this service to set the value of `FringeCodingMode`.

> Note: This service is only available for PRO S (V4), PRO M (V4), Nano (V3), and Pro XS (V3).

This service has one parameter:

- `value` (string): the `FringeCodingMode` value to be set. Values include "Fast" and "Accurate".

Example: set the value of `FringeCodingMode` to "Accurate"

  ```bash
  rosservice call /set_projector_fringecodingmode 'Accurate'
  ```

### [set_projector_powerlevel](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetProjectorPowerLevel.srv)

Invoke this service to obtain the current `powerLevel` value. 

> Note: This service is only available for DLP cameras, excluding Deep (V3) and Pro L Enhanced (V3).

This service has one parameter:

- `value` (string): the `powerLevel` value to be set. Values include "Low", "Normal", and "High".

Example: set the value of `powerLevel` to "High"

  ```bash
  rosservice call /set_projector_powerlevel 'High'
  ```

### [set_uhp_settings](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetUhpSettings.srv)

Invoke this service to set the UHP camera settings.

This service has two parameters:

- `capture_mode` (string): the `UhpCaptureMode` value to be set. Values include "Camera1", "Camera2", and "Merge".
- `fringe_coding_mode` (string): the `UhpFringeCodingMode` value to be set. Values include "Fast" and "Accurate".

Example: set the UHP camera settings to "Merge" and "Accurage"

  ```bash
  rosservice call /set_uhp_settings 'Merge' 'Accurate'
  ```

### [set_uhp_capture_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetUhpCaptureMode.srv)

Invoke this service to set the value of `UhpCaptureMode`.

This service has one parameter:

- `capture_mode` (string): the `UhpCaptureMode` value to be set. Values include "Camera1", "Camera2", and "Merge".

Example: set the value of `UhpCaptureMode` to "Merge"

  ```bash
  rosservice call /set_uhp_capture_mode 'Merge'
  ```

### [set_uhp_fringe_coding_mode](https://github.com/MechMindRobotics/mecheye_ros_interface/blob/master/srv/SetUhpFringeCodingMode.srv)

Invoke this service to set the value of `UhpFringeCodingMode`.

This service has one parameter:

- `fringe_coding_mode` (string):  the `UhpFringeCodingMode` value to be set. Values include "Fast" and "Accurate".

Example: set the value of `UhpFringeCodingMode` to "Accurate"

  ```bash
  rosservice call /set_uhp_fringe_coding_mode 'Accurate'
  ```
