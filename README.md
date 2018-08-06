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

4. Install SteamVR and dependencies

   1. Launch the Steam application and log in with an account
   2. Click on Library > VR
   3. Select SteamVR and click install
   4. Follow instructions up to USB DEVICE REQUIREMENTS:  
   [SteamVR for Linux setup](https://github.com/ValveSoftware/SteamVR-for-Linux)  
   Note: This node was tested on nvidia-390 drivers.
   
3. Clone kuka_iiwa_interface into catkin workspace  
   `git clone https://github.com/UM-ARM-Lab/kuka_iiwa_interface.git`  
4. Clone arc_utilities into catkin_workspace  
   `git clone https://github.com/UM-ARM-Lab/arc_utilities.git`  
5. Clone this repository into catkin_workspace
   `git clone https://github.com/UM-ARM-Lab/vive_teleop.git`  
9. Build the entire catkin workspace

## Usage
1. Connect HTC Vive to computer
   1. If running for first time, start the SteamVR room setup 

Remember to have your catkin workspace sourced and cd into it for each of the below steps. (example: `source ~/catkin_ws/devel/setup.bash && cd ~/catkin_ws`)

2. In a new terminal, run `roscore`
3. In a new terminal, run `rviz`
4. In a new terminal, run `rosrun victor_moveit_config demo.launch`
5. In a new terminal, cd into the catkin workspace and run dual_arm_teleop using 
`~/.steam/steam/ubuntu12_32/steam-runtime/run.sh devel/lib/dual_arm_teleop/dual_arm_teleop_node`

## Troubleshooting
* If SteamVR is having trouble detecting the Vive headset, try switching on/off the SteamVR beta
   1. found by right clicking SteamVR > Properties > Betas >  Select the beta you would like to opt into > beta - SteamVR Beta Update
