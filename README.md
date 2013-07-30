openni2_laserscan
=================

Light-Weight OpenNI2 to ROS.org LaserScan.msg transformation for devices where the full openni_camera and openni_launch package is not working / compiling or its to slow.

Installation:
1. Install OpenNI2
2. $ git clone https://github.com/Flos/openni2_laserscan.git
3. Adapt the include_directories in the CMakeLists.txt to match the include path of your OpenNI2 installation
4. $ catkin_make

Run:
1. create transforms if needed for frame "openni_laserscan" subscripe to topic openni_laserscan
2. $ roscore
3. $ rosrun openni2_laserscan openni2_laserscan_node

