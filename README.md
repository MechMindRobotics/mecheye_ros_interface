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

### Instruction for Ubuntu 16.04

- Download this ROS interface 

  ```bash
  cd YOUR_WORKSPACE_FOLDER (~/ros_ws/src for example)   
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface.git && cd mecheye_ros_interface
  mv CMakeLists_ubuntu16.txt CMakeLists.txt   # Switch to the Ubuntu16 CMakeLists
  export MECHEYE_PATH=${PWD}  # Add present working directory to environment variable
  ```
  
 - Install dependencies

  ```bash
   sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev 
   sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
  ```

- Install OpenCV according to the [official documentation](https://docs.opencv.org/4.1.1/d7/d9f/tutorial_linux_install.html)

  ```bash
  mkdir -p $MECHEYE_PATH/3rdparty/src
  cd $MECHEYE_PATH/3rdparty/src
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git
  cd opencv
  mkdir build && cd build
  cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=$MECHEYE_PATH/3rdparty/opencv4 ..
  make -j7
  make install
  ```


### Instruction for Ubuntu 18.04

- Download this ROS interface

  ```bash
  mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface
  cd ~/ros_ws && catkin_make
  ```

- Install dependencies

  ``` bash
  sudo apt install libzmq5 libzmq3-dev
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
