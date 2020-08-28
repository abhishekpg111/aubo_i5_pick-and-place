# pick_and_place
This project is a vision based pick and place operation of 2D objects using aubo-i5 robot with intel realsense D435 depth camera.

## 1. Development Environment
- Ubuntu 16.04.2
- ROS Kinetic

## 2. ROS Package Dependencies
- aubo_robot
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [find_object_2d](http://wiki.ros.org/find_object_2d)

__Step 1: Install the dependencies__

Install the mentioned ROS packages.

Clone the aubo_robot and realsense-ros packages to your catkin/src folder.

Install the realsense wrappers as described [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

__Step 2: Configure your robot__

Edit the ip address your aubo robot in the _aubo_gripper_moveit_config/launch/moveit.launch_ launch file

__Step 3: Save the images of the objects__

save the images of the objects needs to be picked in the train folder

__Step 4: Run the launch file__

launch the find_2d.launch to start the picking operation. 

    roslaunch pick_and_place pick_2d.launch
<p align="center">    
<img src="picking.gif" align="center" width="70%" height="70%">
</p>

