# Vive Teleop
Vive teleop control for Victor.

## Installation
1. Install ROS

   Follow all the instructions to install ROS Kinetic. Please make sure you have followed all steps, including calls to rosdep.  
   [ROS Install Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) 

2. Install MoveIt!

   Follow all the instructions to install MoveIt! Please make sure you are following the instructions for ROS Kinetic.  
   [MoveIt! Install instructions](http://moveit.ros.org/install/)

3. Install Steam

   Launch the debian installer to install Steam.  
   [Steam download](https://store.steampowered.com/about/)

4. Install SteamVR

   1. Launch the Steam application and log in with an account
   2. Click on Library > VR
   3. Select SteamVR and click install
   
3. Clone kuka_iiwa_interface into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/kuka_iiwa_interface.git`  
4. Clone arc_utilities into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/arc_utilities.git`  
5. Clone this repository into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/vive_teleop.git`
6. Build the entire catkin workspace

## Usage
1. Connect HTC Vive to computer

   If running for first time, start the SteamVR room setup.

Remember to have your catkin workspace sourced and cd into it for each of the below steps. (example: `source ~/catkin_ws/devel/setup.bash && cd ~/catkin_ws`)

2. In a new terminal, run `roscore`
3. In a new terminal, run openvr_ros_driver using 
`~/.steam/steam/ubuntu12_32/steam-runtime/run.sh build/vive_teleop/openvr_ros_driver/openvr_ros_driver_node`
4. In a new terminal, run dual_arm_teleop using 
`roslaunch dual_arm_teleop dual_arm_teleop_node.launch`

### Controls
![alt-text][vive-controller-layout]

The controls for operating the robot are as follows:
1. **Menu button** - Position lock
2. **Trackpad**  
   Vertical axis - motion scaling adjustment while touching  
   Button click - Toggle on/off teleop control
3. **System button**
4. **Status light**
5. **Micro-USB port**
6. **Tracking sensor**
7. **Trigger** - Gripper control
8. **Grip button** - Orientation reset


## Troubleshooting
* If SteamVR is having trouble detecting the Vive headset, try switching on/off the SteamVR beta

   Found by right clicking SteamVR > Properties > Betas >  Select the beta you would like to opt into > beta - SteamVR Beta Update
* Read the SteamVR for Linux guide to make sure everything is configured properly

   [SteamVR for Linux guide](https://github.com/ValveSoftware/SteamVR-for-Linux)



[vive-controller-layout]: https://www.vive.com/media/filer_public/17/5d/175d4252-dde3-49a2-aa86-c0b05ab4d445/guid-2d5454b7-1225-449c-b5e5-50a5ea4184d6-web.png "Vive Controller Layout"
