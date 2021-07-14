Mech-Eye ROS Interface
====================
Official ROS interface for Mech-Eye cameras.

<http://www.mech-mind.net/>

## Dependencies:
- ZeroMQ   >= 4.2.1
- Protobuf >= 3
- OpenCV   >= 3
- PCL 	   >= 1.7
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

## Instruction for Ubuntu 18.04 (ROS Melodic) and Ubuntu 20.04 (ROS Noetic):
- `sudo apt install libzmq5 libzmq3-dev`
- build project with `catkin build`


## Instruction for Ubuntu 16.04 (ROS Kinetic):
- Set up mecheye ros interface path:
```bash
cd YOUR_PACKAGE_FOLDER
export MECHEYE_PATH=${PWD}
mv CMakeLists_ubuntu16.txt CMakeLists.txt
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

- Build project with `catkin build`

## Take picture and publish it to ROS message
1. run the camera: `roslaunch mecheye_ros_interface start_camera.launch`
2. take a picture: `rosservice call /run_mechmind_camera`
