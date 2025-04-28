# Migration Guide from Version 2.0.2 to Version 2.3.4

As Mech-Eye API has been restructured in version 2.2.0, the ROS 1 interface for Mech-Eye Industrial 3D Camera has also been restructured accordingly. This document lists the topics and services that have changed from the previous release of the ROS 1 interface to the latest release. You can modify your program according to this document.

## Compatibility

* Previous release of the ROS 1 interface: compatible with Mech-Eye SDK 2.0.1 and 2.0.2
* Latest release of the ROS 1 interface: compatible with Mech-Eye SDK 2.3.4

If you would like to use the latest ROS 1 interface for the camera, please install the latest version of Mech-Eye SDK or upgrade Mech-Eye SDK to the latest version.

If you are using Mech-Eye SDK 2.0.2 and below, please install the previous release of the ROS 1 interface (can be found from the [Releases page](https://github.com/MechMindRobotics/mecheye_ros_interface/releases) on GitHub.)

>Note: No compatible release of ROS 1 interface was made for Mech-Eye SDK 2.1.0 to 2.3.3. If you are using these versions of Mech-Eye SDK and would like to use the ROS 1 interface, please upgrade Mech-Eye SDK to the latest version.

## Topics

### Depth Map

* Version 2.0.2: /mechmind/depth_image
* Version 2.3.4: /mechmind/depth_map

### Textured Point Cloud

* Version 2.0.2: /mechmind/color_point_cloud
* Version 2.3.4: /mechmind/textured_point_cloud

## Services

### Data Acquisition

#### Capture 2D Image

* Version 2.0.2:

  ```bash
  rosservice call /capture_color_map
  ```

* Version 2.3.4:

  ```bash
  rosservice call /capture_color_image
  ```

#### Capture Textured Point Cloud

* Version 2.0.2:

  ```bash
  rosservice call /capture_color_point_cloud
  ```

* Version 2.3.4:

  ```bash
  rosservice call /capture_textured_point_cloud
  ```

### Adjust Camera Parameters

#### Get Scan2DExpectedGrayValue

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_expected_gray_value
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_int_parameter "{name: Scan2DExpectedGrayValue}"
  ```

#### Set Scan2DExpectedGrayValue

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_expected_gray_value "{value: 20}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_int_parameter "{name: Scan2DExpectedGrayValue, value:20}"
  ```

#### Get Scan2DExposureMode

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_exposure_mode
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: Scan2DExposureMode}"
  ```

#### Set Scan2DExposureMode

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_exposure_mode "{value: HDR}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: Scan2DExposureMode, value: HDR}"
  ```

#### Get Scan2DHDRExposureSequence

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_exposure_sequence
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_float_array_parameter "{name: Scan2DHDRExposureSequence}"
  ```

#### Set Scan2DHDRExposureSequence

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_exposure_sequence "{sequence: [30.0,35.5,40.0]}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_float_array_parameter "{name: Scan2DHDRExposureSequence, array: [30.0, 35.5, 40.0]}"
  ```

#### Get Scan2DExposureTime

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_exposure_time
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_float_parameter "{name: Scan2DExposureTime}"
  ```

#### Set Scan2DExposureTime

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_exposure_time "{value: 35.5}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_float_parameter "{name: Scan2DExposureTime, value: 35.5}"
  ```

#### Get Scan2DROI

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_roi
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_roi_parameter "{name: Scan2DROI}"
  ```

#### Set Scan2DROI

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_roi "{x: 20, y: 20, width: 600, height: 800}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_roi_parameter "{name: Scan2DROI, x: 20, y: 20, width: 600, height: 800}"
  ```

#### Get Scan2DSharpenFactor

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_sharpen_factor
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_float_parameter "{name: Scan2DSharpenFactor}"
  ```

#### Set Scan2DSharpenFactor

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_sharpen_factor "{value: 0.5}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_float_parameter "{name: Scan2DSharpenFactor, value: 0.5}"
  ```

#### Get Scan2DToneMappingEnable

* Version 2.0.2:

  ```bash
  rosservice call /get_2d_tone_mapping
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_bool_parameter "{name: Scan2DToneMappingEnable}"
  ```

#### Set Scan2DToneMappingEnable

* Version 2.0.2:

  ```bash
  rosservice call /set_2d_tone_mapping "{value: True}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_bool_parameter "{name: Scan2DToneMappingEnable, value: True}"
  ```

#### Get Scan3DExposureSequence

* Version 2.0.2:

  ```bash
  rosservice call /get_3d_exposure
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_float_array_parameter "{name: Scan3DExposureSequence}"
  ```

#### Set Scan3DExposureSequence

* Version 2.0.2:

  ```bash
  rosservice call /set_3d_exposure "{sequence: [30.0,35.5,40.0]}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_float_array_parameter "{name: Scan3DExposureSequence, array: [30.0, 35.5, 40.0]}"
  ```

#### Get Scan3DGain

* Version 2.0.2:

  ```bash
  rosservice call /get_3d_gain
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_float_parameter "{name: Scan3DGain}"
  ```

#### Set Scan3DGain

* Version 2.0.2:

  ```bash
  rosservice call /set_3d_gain "{value: 2.5}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_float_parameter "{name: Scan3DGain, value: 2.5}"
  ```

#### Get Scan3DROI

* Version 2.0.2:

  ```bash
  rosservice call /get_3d_roi
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_roi_parameter "{name: Scan3DROI}"
  ```

#### Set Scan3DROI

* Version 2.0.2:

  ```bash
  rosservice call /set_3d_roi "{x: 20, y: 20, width: 600, height: 800}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_roi_parameter "{name: Scan3DROI, x: 20, y: 20, width: 600, height: 800}"
  ```

#### Get PointCloudOutlierRemoval

* Version 2.0.2:

  ```bash
  rosservice call /get_cloud_outlier_filter_mode
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: PointCloudOutlierRemoval}"
  ```

#### Set PointCloudOutlierRemoval

* Version 2.0.2:

  ```bash
  rosservice call /set_cloud_outlier_filter_mode '!!str Off'
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: PointCloudOutlierRemoval, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get PointCloudSurfaceSmoothing

* Version 2.0.2:

  ```bash
  rosservice call /get_cloud_smooth_mode
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: PointCloudSurfaceSmoothing}"
  ```

#### Set PointCloudSurfaceSmoothing

* Version 2.0.2:

  ```bash
  rosservice call /set_cloud_smooth_mode '!!str Off'
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: PointCloudSurfaceSmoothing, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get DepthRange

* Version 2.0.2:

  ```bash
  rosservice call /get_depth_range
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_range_parameter "{name: DepthRange}"
  ```

#### Set DepthRange

* Version 2.0.2:

  ```bash
  rosservice call /set_depth_range "{lower: 300, upper: 1000}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_range_parameter "{name: DepthRange, lower: 300, upper: 1000}"
  ```

#### Get FringeContrastThreshold

* Version 2.0.2:

  ```bash
  rosservice call /get_fringe_contrast_threshold
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_int_parameter "{name: FringeContrastThreshold}"
  ```

#### Set FringeContrastThreshold

* Version 2.0.2:

  ```bash
  rosservice call /set_fringe_contrast_threshold "{value: 3}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_int_parameter "{name: FringeContrastThreshold, value: 3}"
  ```

#### Get FringeMinThreshold

* Version 2.0.2:

  ```bash
  rosservice call /get_fringe_min_threshold
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_int_parameter "{name: FringeMinThreshold}"
  ```

#### Set FringeMinThreshold

* Version 2.0.2:

  ```bash
  rosservice call /set_fringe_min_threshold "{value: 3}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_int_parameter "{name: FringeMinThreshold, value: 3}"
  ```

#### Get LaserPowerLevel

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /get_laser_settings
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_int_parameter "{name: LaserPowerLevel}"
  ```

#### Set LaserPowerLevel

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /set_laser_settings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_int_parameter "{name: LaserPowerLevel, value: 60}"
  ```

#### Get LaserFringeCodingMode

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /get_laser_settings
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: LaserFringeCodingMode}"
  ```

#### Set LaserFringeCodingMode

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /set_laser_settings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: LaserFringeCodingMode, value: Fast}"
  ```

#### Get LaserFrameRange

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /get_laser_settings
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_range_parameter "{name: LaserFrameRange}"
  ```

#### Set LaserFrameRange

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /set_laser_settings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_range_parameter "{name: LaserFrameRange, lower: 20, upper: 80}"
  ```

#### Get LaserFramePartitionCount

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /get_laser_settings
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_int_parameter "{name: LaserFramePartitionCount}"
  ```

#### Set LaserFramePartitionCount

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  rosservice call /set_laser_settings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_int_parameter "{name: LaserFramePartitionCount, value: 2}"
  ```

#### Get AntiFlickerMode

* Version 2.0.2:

  ```bash
  rosservice call /get_projector_anti_flicker_mode
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: AntiFlickerMode}"
  ```

#### Set AntiFlickerMode

* Version 2.0.2:

  ```bash
  rosservice call /set_projector_anti_flicker_mode '!!str Off'
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: AntiFlickerMode, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get ProjectorFringeCodingMode

* Version 2.0.2:

  * Models other than UHP:

    ```bash
    rosservice call /get_projector_fringe_coding_mode
    ```

  * UHP:

    ```bash
    rosservice call /get_uhp_fringe_coding_mode
    ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: ProjectorFringeCodingMode}"
  ```

#### Set ProjectorFringeCodingMode

* Version 2.0.2:

  * Models other than UHP:

    ```bash
    rosservice call /set_projector_fringe_coding_mode "{value: Accurate}"
    ```

  * UHP:

    ```bash
    rosservice call /set_uhp_fringe_coding_mode "{value: Accurate}"
    ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: ProjectorFringeCodingMode, value: Accurate}"
  ```

#### Get ProjectorPowerLevel

* Version 2.0.2:

  ```bash
  rosservice call /get_projector_power_level
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: ProjectorPowerLevel}"
  ```

#### Set ProjectorPowerLevel

* Version 2.0.2:

  ```bash
  rosservice call /set_projector_power_level "{value: High}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: ProjectorPowerLevel, value: High}"
  ```

#### Get UhpCaptureMode

* Version 2.0.2:

  ```bash
  rosservice call /get_uhp_capture_mode
  ```

* Version 2.3.4:

  ```bash
  rosservice call /get_enum_parameter "{name: UhpCaptureMode}"
  ```

#### Set UhpCaptureMode

* Version 2.0.2:

  ```bash
  rosservice call /set_uhp_capture_mode "{capture_mode: Merge}"
  ```

* Version 2.3.4:

  ```bash
  rosservice call /set_enum_parameter "{name: UhpCaptureMode, value: Merge}"
  ```
