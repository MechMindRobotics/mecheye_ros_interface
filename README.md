Mech-Eye ROS Interface
====================
Official ROS interface for Mech-Eye cameras.

<http://www.mech-mind.net/>

## Dependencies:
- ZeroMQ   >= 4.2.1
- Protobuf == 3.6.1
- OpenCV   >= 3
- PCL 	   >= 1.7 or 1.8
- Eigen    3.3.0
- VTK      6.3.0

## How to use interface functions of Mech-Eye cameras:
- Interface functions are declared in ```CameraClient.h```.
- Connect to camera via specific ip address.
- Call other functions.


## How to build and run sample project:

- Download or build dependency files.
- Build with CMake.
- Run with one Mech-Eye camera.


Note that `/homeL/lhz` will be changed to your home dir manually in `CMakeList.txt`.

## ZeroMQ install guide
- `sudo apt install libzmq5 libzmq3-dev` (only work for ubuntu 18.04)
- for ubuntu 16.04, install from source code:

```bash
wget https://github.com/zeromq/libzmq/releases/download/v4.3.2/zeromq-4.3.2.zip
unzip zeromq-4.3.2.zip
cd zeromq-4.3.2
./autogen.sh
./configure --prefix=/homeL/lhz/code/eye_ws/src/mecheye_ros_interface/3rdparty/libzmq
make -j7
make install
```

## OpenCV4 install guide
`OpenCV 4` can be installed according to official documentation: https://docs.opencv.org/4.1.1/d7/d9f/tutorial_linux_install.html

To avoid version confusion of OpenCV, you should firstly set OpenCV_DIR before find_package(OpenCV REQUIRED) in CMakeLists.txt, e.g.
`SET("OpenCV_DIR" "3rdparty/opencv4/lib/cmake/opencv4")`, if `opencv4` is installed in `mecheye_ros_interface/3rdparty/opencv4`

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

mkdir -p ~/code/eye_ws/src/mecheye_ros_interface/3rdparty/src
cd ~/code/eye_ws/src/mecheye_ros_interface/3rdparty/src
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/homeL/lhz/code/eye_ws/src/mecheye_ros_interface/3rdparty/opencv4 ..
make -j7
make install
```

## Protobuf install guide:

```bash
cd ~/code/eye_ws/src/mecheye_ros_interface/3rdparty/src
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.zip
unzip protobuf-all-3.6.1.zip
cd protobuf-3.6.1
./configure --prefix=/homeL/lhz/code/eye_ws/src/mecheye_ros_interface/3rdparty/protobuf
make -j7
make check
make install
sudo ldconfig # refresh shared library cache.
```

## instruction for ubuntu 18.04:
- rename `CMakeLists_ubuntu18.txt` to `CMakeLists.txt`
- only need to install `protobuf` from source
