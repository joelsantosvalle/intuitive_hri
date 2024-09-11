# Intuitive Human Robot Cooperation

## Requierments and supported platforms
### Supported platforms/releases:

|Platform|ROS Release|
|-|------|
|[Ubuntu 22.04](https://releases.ubuntu.com/20.04/)|[ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)|

### Requirements: 

You should have the [Leap Motion SDK](https://developer.leapmotion.com/tracking-software-download), and [UR robot driver](https://docs.ros.org/en/rolling/p/ur_robot_driver/installation/toc.html) installed on your device. Additional we recommend that you have [terminator](https://wiki.ubuntuusers.de/Terminator/) installed so you can open multiple terminals in the same window

## Documentation and Enviornment setup
For guidance on installation, development, environmental setup, and troubleshooting, see our [documentation](). This documentation includes a description of the hardware which was used to develop and test this software.

## Current features
[Pick up of an object/piece and delivery of this object to the hand operator through information acquired by the leap motion sensor](/src/leapmotion_robot_control)

## Installation of this package (it is assumed you already have the all the necessary robot and gripper drivers installed)

**1.** Install the Leap Motion sensor linux drivers with their [guide.](https://docs.ultraleap.com/linux/)

**2.** Go to the source folder of your catkin workspace.
```bash 
cd ~/your_catkin_ws/src
git clone https://github.com/joelsantosvalle/intuitive_hri.git
cd ~/your_catkin_ws
colcon build --packages-select intuitive_hri
```
**3.** Source your current catkin workspace (Optional in case your workspace is not sourced in your bashrc)
```bash 
source ~/catkin_ws/devel/setup.bash
```
## Running the scripts
If you didn't add `source $YOUR_WORKSPACE/devel/setup.bash` and `source /opt/ros/humble/setup.bash` to your `.bashrc`, remember to source it when you open a new terminal. Also, in the following example catkin_ws is the name of our workspace, but this could change depending on the name you gave to your workspace.

### Example
#### Visualizing the Leap motion sensor (Optinal)
1. Open a terminal and type:
```sh
ultraleap-hand-tracking-control-panel
```
#### Using the Leap Motion sensor
2. Open a terminal, source it and type the following:
```sh
cd ~/catkin_ws/src/intuitive_hri/src/leapmotion_robot_control/
gcc -o LeapListener LeapMotionListener.c -I/usr/include -L/usr/lib/ultraleap-hand-tracking-service/libLeapC.so -l LeapC && ./LeapListener 
```
#### Publishing the data extracted from the Leap motion sensor into ROS, transforming sensor to robot coordinates and controlling the robot
3. Open a new terminal, source it and type the following:
```sh
cd ~/catkin_ws
ros2 launch intuitive_hri leap_robot_control.launch
```

### Example Video
If all the previous steps were followed correctly and carefully you should be able to see the [following.](https://youtu.be/oda8lf_sLHQ)


