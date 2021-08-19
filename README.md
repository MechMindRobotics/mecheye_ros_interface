Mech-Eye ROS Interface
====================
Official ROS interface for Mech-Eye cameras. If the MechEye camera version is before v1.2.0, please download sdk of previous versions, see tags.

<http://www.mech-mind.net/>

## Dependencies:
- ZeroMQ   >= 4.2.1
- OpenCV   >= 3
- PCL      >= 1.7 or 1.8
- Eigen    3.3.0
- VTK      6.3.0

## How to use interface functions of Mech-Eye cameras:
- Interface functions are declared in ```CameraClient.h```.
- Connect to camera via specific ip address.
- Call other functions.


## How to build and run sample project:

- Download or build dependency files.
- Build with `catkin build`.
- Run with one Mech-Eye camera.

## Instruction for ubuntu 16.04:
- Set up mecheye ros interface path:
```bash
cd YOUR_PACKAGE_FOLDER
mv CMakeLists_ubuntu16.txt CMakeLists.txt
export MECHEYE_PATH=${PWD}
mkdir -p $MECHEYE_PATH/3rdparty/src
```

- ZeroMQ install guide

```bash
cd $MECHEYE_PATH/3rdparty/src
wget https://github.com/zeromq/libzmq/releases/download/v4.3.2/zeromq-4.3.2.zip
unzip zeromq-4.3.2.zip
cd zeromq-4.3.2
./autogen.sh
./configure --prefix=$MECHEYE_PATH/3rdparty/libzmq
make -j7
make install
```

- OpenCV4 install guide
`OpenCV4` can be installed according to official documentation: https://docs.opencv.org/4.1.1/d7/d9f/tutorial_linux_install.html

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

cd $MECHEYE_PATH/3rdparty/src
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=$MECHEYE_PATH/3rdparty/opencv4 ..
make -j7
make install
```

- Protobuf install guide:

```bash
cd $MECHEYE_PATH/3rdparty/src
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.zip
unzip protobuf-all-3.6.1.zip
cd protobuf-3.6.1
./configure --prefix=$MECHEYE_PATH/3rdparty/protobuf
make -j7
make check
make install
export PATH=$MECHEYE_PATH/3rdparty/protobuf/bin:$PATH
```

## Instruction for ubuntu 18.04:
- `sudo apt install libzmq5 libzmq3-dev`
- `mkdir -p ~/ros_ws/src`
- `cd ~/ros_ws/src`
- `git clone https://github.com/MechMindRobotics/mecheye_ros_interface`
- `cd ~/ros_ws && catkin_make`
- Change config in `~/ros_ws/src/mecheye_ros_interface/launch/start_camera.launch`
    - save_file: `true` to enable save file, otherwise keep to false
    - camera_ip: change to your camera ip address here
    - Image save path can be changed in source code `/mecheye_ros_interface/src/main.cpp`.
- Run `roslaunch mecheye_ros_interface start_camera.launch`. Then, the camera will start working.
- Open a new terminal, run `rosservice call /run_mechmind_camera` to take a picture.
