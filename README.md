# VR teleop for unity:

## Installation

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



## Usage
1. Connect HTC Vive to computer (or plug in battery if using wireless)
2. Launch SteamVR
   1. Launch Steam
   2. Click on Library > VR
   3. Select SteamVR and click play
   
   If you are running SteamVR for first time, please start the SteamVR room setup to calibrate the device.
   
2. (If using wireless) Launch VIVE Wireless

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
