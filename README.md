# UAV Hand gestures

This ROS node is capable of control a UAV (Parrot Bebop) with Tiny YOLO v3 and a Kinect Sensor (V1). This approach consists in the detection of a particular gesture based on ASL to command the UAV and navigate through the different waypoints created by the user.

## Prerequisites

This node was tested using Ubuntu 18.04 and ROS Melodic

+ PID Controller ROS
+ Vicon Bridge
+ Bebop_ROS_Examples
+ OpenNI launch
+ OpenNI Tracker
+ NITE 1.5.2
+ Bebop Autonomy
+ Darknet ROS
+ CUDA

## Installation

1. Create a new workspace and move in it

``$ mkdir catkin_ws && cd catkin_ws``

2. Create a source file and move into it

``$ mkdir src && cd src``

3. Install the following resources from binaries

- PID Controller:

``$ sudo apt-get install ros-melodic-pid``

- CUDA:

``$ ubuntu-drivers devices``

If you agree with the recommendation:

``$ sudo ubuntu-drivers autoinstall``

Finally, reboot.

3. Install the rest of the packages from source into the created workspace

- Vicon:

``$ git clone https://github.com/ethz-asl/vicon_bridge``

``$ cd ../``

``$ catkin_make``


Important: The address of the vicon system should be modified (at the launch file) to connect the node with the motion camera system.


- OpenNI Launch:

``$ cd src``

``$ git clone https://github.com/ros-drivers/openni_camera.git ``

``$ cd ../``

``$ catkin_make``

- OpenNI Tracker

``$ cd src``

``$ git clone https://github.com/ros-drivers/openni_tracker.git``

``$ cd ../``

``$ catkin build openni_tracker``


- NITE 1.5.2

``$ cd src``

``$ git clone https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23.git``

``$ cd NITE-Bin-Dev-Linux-v1.5.2.23/x64``

``$ sudo ./install.sh``

``$ cd ../../``

- Bebop Autonomy:

``$ cd src``

``$ sudo apt-get install build-essential python-rosdep python-catkin-tools``

``$ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy``

``$ rosdep update``

``$ rosdep install --from-paths src -i``

``$ cd ../``

``$ catkin build``


- Bebop_ROS_Examples:

``$ cd src``

``$ git clone https://github.com/TOTON95/Bebop_ROS_Examples.git``

``$ catkin_make``

``$ cd ../``


- Darknet ROS:

``$ cd src``

``$ git clone --recursive git@github.com:leggedrobotics/darknet_ros.git``

``$ cd ../``

``$ catkin_make -DCMAKE_BUILD_TYPE=Release``

If the compiler throws errors like:

**nvcc fatal : Unsupported gpu architecture 'compute_61'.**

That means that the configuration of the graphical card is not at the CMakeLists.txt file, look for the compute capibility of your hardware in sites like Wikipedia and add the missing line, like this:

``-O3 -gencode arch=compute_62,code=sm_62``

(Please refer to https://github.com/leggedrobotics/darknet_ros, for more information)

Follow the instructions at the website above to locate the weights of the hand gestures


Finally clone this repository and build it 

``git clone https://github.com/TOTON95/Hand_Gesture_UAV.git``

``cd ../..``

``catkin_make``


## Running 

- Start a new terminal and type roscore
- Start a new terminal for each package and type ``rosrun <name_package> <name_node>``, being the hand gestures the last one to activate
- Some of the packages offer the possibility to be initialized using roslaunch (see the respective repository for more info)


