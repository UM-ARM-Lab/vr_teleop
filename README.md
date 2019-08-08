# VR teleop for unity:


Terminal 1:
```
ssh loki
roscore
```

Terminal 2:
```
ssh realtime
export ROS_MASTER_URI=http://loki.local:11311
roslaunch victor_hardware_interface dualarm_lcm_bridge.launch
```

Terminal 3:
```
ssh loki
roslaunch mps_launch_file kinect_vicon_real_robot.launch pov:="victor_head" fps_limit:="10"
```

Terminal 4:
```
export ROS_MASTER_URI=http://loki.local:11311
rosrun dual_arm_teleop make_victor_fast.py
roslaunch unity_launch_files unity_setup.launch
```





# Old (For installing without unity).......

# VR Teleop
This repository contains a pipeline enabling users to control the Victor robot with virtual reality devices. Currently, support for virtual reality devices extends only to the HTC Vive, but in theory other devices should be compatible with minor tweaking.

## Demo
[![Victor Demo](https://i.imgur.com/OlLNeAE.png)](https://youtu.be/6RulnpVsO-8)
## Installation
Note: this pipeline has been tested on a GeForce video card running nvidia-390 drivers, on Ubuntu 16.04. Other setups may or may not work out of the box.

1. Install ROS

   Follow all the instructions to install ROS Kinetic. Please make sure you have followed all steps, including calls to rosdep.  
   [ROS Install Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) 

2. Install MoveIt!

   Follow all the instructions to install MoveIt! Please make sure you are following the instructions for ROS Kinetic.  
   [MoveIt! Install instructions](http://moveit.ros.org/install/)
   
   Install the moveit-visual-tools: `sudo apt-get install ros-kinetic-moveit-visual-tools`

3. Install Steam

   Launch the debian installer to install Steam.  
   [Steam download](https://store.steampowered.com/about/)

4. Install SteamVR

   1. Launch the Steam application and log in with an account
   2. Click on Library > VR
   3. Select SteamVR and click install
   
   You can also install SteamVR using Steam's browser protocol (enter this link into your browser's address bar): `steam://install/250820`
   
3. Clone kuka_iiwa_interface into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/kuka_iiwa_interface.git`  
4. Clone arc_utilities into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/arc_utilities.git`  
5. **Recursively** clone this repository into catkin workspace  
   `git clone --recurse-submodules https://github.com/UM-ARM-Lab/vr_teleop.git`
6. Build the entire catkin workspace

## Usage
1. Connect HTC Vive to computer
2. Launch SteamVR
   1. Launch Steam
   2. Click on Library > VR
   3. Select SteamVR and click play
   
   If you are running SteamVR for first time, please start the SteamVR room setup to calibrate the device.

Remember to have your catkin workspace sourced and cd into it for each of the below steps. (example: `source ~/catkin_ws/devel/setup.bash && cd ~/catkin_ws`)

3. In a new terminal, run `roscore`
4. In a new terminal, run openvr_ros_driver using  
   `~/.steam/steam/ubuntu12_32/steam-runtime/run.sh build/vr_teleop/openvr_ros_driver/openvr_ros_driver_node`
5. In a new terminal, run dual_arm_teleop using  
   `roslaunch dual_arm_teleop dual_arm_teleop_node.launch`
6. In a new terminal, run rviz using  
   `rviz src/vr_teleop/dual_arm_teleop/launch/moveit.rviz`

At this point, you can either launch the real victor or the fake victor. The teleop node publishes commands in impedance mode.

### Controls
![alt-text][vive-controller-layout]

The controls for operating the robot are as follows:
1. **Menu button** - Position lock
2. **Trackpad**  
   Button click - Toggle on/off teleop control
3. **System button**
4. **Status light**
5. **Micro-USB port**
6. **Tracking sensor**
7. **Trigger** - Gripper control
8. **Grip button** - Orientation reset


## Troubleshooting
* If SteamVR is having trouble detecting the Vive headset, try switching on/off the SteamVR beta

   Found by right clicking SteamVR > Properties > Betas >  Select the beta you would like to opt into
* Read the SteamVR for Linux guide to make sure everything is configured properly

   [SteamVR for Linux guide](https://github.com/ValveSoftware/SteamVR-for-Linux)

* If Unity is not detecting any input from ROS (kinematics, kinect), check the ip address match between the Unity RosConnector node and the ubuntu VM (run `ifconfig`)


[vive-controller-layout]: https://www.vive.com/media/filer_public/17/5d/175d4252-dde3-49a2-aa86-c0b05ab4d445/guid-2d5454b7-1225-449c-b5e5-50a5ea4184d6-web.png "Vive Controller Layout"
