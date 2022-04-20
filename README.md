# Mech-Eye ROS Interface

This repository contains the official ROS interface for Mech-Eye camera.

## Installation

### Dependencies

|   Package    |    Version    |
| :----------: | :-----------: |
|    OpenCV    |     >= 3      |
|     PCL      | >= 1.7 or 1.8 |
|    Eigen     |     3.3.0     |
|     VTK      |     6.3.0     |
| Mech-Eye SDK |     1.5.1     |

### Install Mech-Eye SDK

Download and install MechEyeApi_1.5.1 compatible with Ubuntu from this [link](https://www.mech-mind.com/download/camera-sdk.html).

### Install the Interace

- Download this ROS interface

  ```bash
  mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface
  cd ~/ros_ws && catkin_make
  ```

## Brief intro to the interface

- Interface functions can be found in the documentation inside [Mech-Eye SDK](https://www.mech-mind.com/download/camera-sdk.html).
- Change config in `~/ros_ws/src/mecheye_ros_interface/launch/start_camera.launch`
  - save_file: `true` to enable save file, otherwise keep it as `false`
  - camera_ip: change to your camera ip address here (also remember to uncomment the lines in `main.cpp` to connect to a specific camera)
  - at the moment, image save path can only be changed in source code `/mecheye_ros_interface/src/main.cpp`.
  - remember to catkin_make again after changing `main.cpp`.
- Source the build workspace

  ```bash
  source ~/ros_ws/devel/setup.bash
  ```

  Run

  ```bash
  roslaunch mecheye_ros_interface start_camera.launch 
  ```

  Then, the camera will start working.
- Open a new terminal, source the workspace and run

  ```bash
  rosservice call /run_mechmind_camera
  ```

- Select a camera in LAN to connect and capture once.
- Call other functions available in the [SDK](https://www.mech-mind.com/download/camera-sdk.html) documentation.
