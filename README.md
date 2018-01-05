# ZED-Camera-ROS
A simply way to run ZED camera on ROS (Jetson TX1)

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, odometry information and supports the use of multiple ZED cameras.
Getting started

    First, download the latest version of the ZED SDK on stereolabs.com
    Download the ZED ROS wrapper here.
    For more information, check out our ROS documentation, our ROS wiki or our blog post. 
    If you want to customize the wrapper, check the ZED API documentation.

Prerequisites

    Ubuntu 16.04
    ZED SDK ≥ 2.1 and its dependencies (OpenCV, CUDA)
    ROS Kinetic
    Point Cloud Library (PCL) with the following dependencies
    
Dependancies for Point Cloud Libraries
   First, install the following dependencies (minimum, use stable option) :
    Boost : OpenNI and OpenNI2.
    Description: Open Natural Interaction, facilitates access and use of devices.
    
    Eigen 3.0
    Description : C++ template library for linear algebra such as matrices, vectors, numerical solvers, and related algorithms.
    
    FLANN 1.7.1
    Description : a library for performing fast approximate nearest neighbor searches in high dimensional spaces. 
    It contains a collection of algorithms and automatically choosing the best algorithm 
    and optimum parameters depending on the dataset.
    
    VTK 5.6 : Visualization ToolKitis an open-source, freely available software system for 3D computer graphics, 
    image processing, and visualization. It consists of a C++ class library and several interpreted interface layers
    including Tcl/Tk, Java, and Python.
 
 Install All Dependancies :
    sudo apt-get install libopenni-dev libopenni2-dev libeigen3-dev libflann-dev libvtk5-dev
 
  Install PCL :
Download stable http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php
Commands on terminal :
  tar xvfj pcl-pcl-1.7.2.tar.gz
  cd pcl-pcl-1.7.2 && mkdir Reease && cd Release
  
Run the CMake build system using the default options (Easy mode):
  cmake ..
Or change them (uses cmake-curses-gui) (Expert mode):
  ccmake ..

Or compilation with compiler optimisation (Stable):
  cmake -DCMAKE_BUILD_TYPE=Release ..

Finnaly compile everything.
  make -j2

And install the result:
  make -j2 install


Build the program

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    roscpp
    rosconsole
    sensor_msgs
    opencv
    image_transport
    dynamic_reconfigure
    urdf

Place the package folder zed_wrapper in your catkin workspace source folder ~/catkin_ws/src.
  git clone https://github.com/eagle-view/ZED-Camera-ROS
Open a terminal and build the package:

  cd ~/catkin_ws/
  catkin_make
  source ./devel/setup.bash

Run the program
To launch the wrapper along with an Rviz preview, open a terminal and launch:

roslaunch zed_wrapper display.launch

To launch the wrapper without Rviz, use:

roslaunch zed_wrapper zed.launch
