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

## Installation of this package
**1.** Go to the source folder of your catkin workspace.
```bash 
cd ~/catkin_ws/src
git clone https://github.com/Jasv06/human_robot_interaction.git
cd ~/catkin_ws
catkin_make
```
**2.** Source your current catkin workspace.
```bash 
source ~/catkin_ws/devel/setup.bash
```
**3.** Extract the LeapMotion folder and place it in your desired location outside your catkin working space.

**4.** Go to the [Leap_client_demonstrator.py](/scripts/demonstrator_hold_hand/Leap_client_demonstrator.py) file and change the path in line 8 based on the location where you placed the folder LeapMotion.

**5.** Go to the [Leap_client_industrial.py](/scripts/Industrial_like_robot/Leap_client_industrial.py) file and change the path in line 8 based on the location where you placed the folder LeapMotion.

## Running the scripts
If you didn't add `source $YOUR_WORKSPACE/devel/setup.bash` and `source /opt/ros/noetic/setup.bash` to your `.bashrc`, remember to source it when you open a new terminal. Also, in the following example catkin_ws is the name of our workspace, but this could change depending on the name you gave to your workspace.

### Example
#### Connecting the Leap motion sensor
1. Open a terminal and type:
```sh
ultraleap-hand-tracking-control-panel
```
#### Use the Leap Motion sensor
2. Open a terminal, source it and type the following:
```sh
cd ~/catkin_ws/src/human_robot_interaction/scripts/Industrial_like_robot
python2 Leap_client_industrial.py
```
#### Publishing the data extracted from the Leap motion sensor into ROS
3. Open a new terminal, source it and type the following:
```sh
cd ~/catkin_ws
roslaunch human_robot_interaction leap_ros_industrial.launch
```
#### Connecting to the Robot
4. Ensure the Robot is connected correctly, open a new terminal, source it and type the following:
```sh
cd ~/catkin_ws
roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=rx150 
```
### Example Video
If all the previous steps were followed correctly and carefully you should be able to see the following. (Click on the image below)

[![Watch the video](images/ur.HEIC)](https://youtu.be/oda8lf_sLHQ)


