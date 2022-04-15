# Mech-Eye ROS Interface

This repository contains the official ROS interface for Mech-Eye SDK v1.5.1.

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

- Set up mecheye ros interface path

  ```bash
  cd YOUR_WORKSPACE_FOLDER (~/ros_ws/src for example)
  ```

- Clone the repository

  ```bash
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface.git && cd mecheye_ros_interface
  ```

- Switch to Ubuntu16 CMakeLists

  ```bash
  mv CMakeLists_ubuntu16.txt CMakeLists.txt
  ```

- Add present working directory to environment variable

  ```bash
  export MECHEYE_PATH=${PWD}
  ```

- Create new directory for dependencies

  ```bash
  mkdir -p $MECHEYE_PATH/3rdparty/src
  ```

- Install packages

  ```bash
  sudo apt-get install build-essential
  ```

  ```bash
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  ```

  ```bash
  sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
  ```

- OpenCV4 install guide. `OpenCV4` can be installed according to [official documentation](https://docs.opencv.org/4.1.1/d7/d9f/tutorial_linux_install.html)

  ```bash
  cd $MECHEYE_PATH/3rdparty/src
  ```

  ```bash
  git clone https://github.com/opencv/opencv.git
  ```

  ```bash
  git clone https://github.com/opencv/opencv_contrib.git
  ```

  ```bash
  cd opencv
  ```

  ```bash
  mkdir build && cd build
  ```

  ```bash
  cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=$MECHEYE_PATH/3rdparty/opencv4 ..
  ```

  ```bash
  make -j7
  ```

  ```bash
  make install
  ```

### Instruction for Ubuntu 18.04

- Install packages

  ``` bash
  sudo apt install libzmq5 libzmq3-dev
  ```

- Create directory for source code

  ```bash
  mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
  ```

- Clone the repository

  ```bash
  git clone https://github.com/MechMindRobotics/mecheye_ros_interface
  ```

- Catkin make

  ```bash
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
