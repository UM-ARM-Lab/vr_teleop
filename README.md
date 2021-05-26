# VR teleop for unity:
This is the ROS portion of the armlabs VR teleopeartion suite. You will also need the [Unity package](https://github.com/UM-ARM-Lab/unity_victor_teleop).
You can find further explanation on the [wiki](https://github.com/UM-ARM-Lab/unity_victor_teleop/wiki)

![flow diagram](flow_diagram.png)

Videos of the Robot operated using this software can be found on Youtube:
- [Making pancakes](https://www.youtube.com/watch?v=EahUsJKVfw8)
- [Filling a shopping bag](https://www.youtube.com/watch?v=EvpxsqET01c)

## Installation
Dependencies
- moveit (planning interface, visual-tools)


## How to run:
(This is a quick reference. See the [tutorials](https://github.com/UM-ARM-Lab/unity_victor_teleop/wiki/Tutorial-1:-Set-up-Unity-package) for more detailed instructions)

You will need 
- A windows computer with unity connected to the Vive with the [Unity package](https://github.com/UM-ARM-Lab/unity_victor_teleop)
- A linux computer (or virtual machine) with ROS
- (optionally) A linux computer connected to the physical robot, exposing the proper interface
- (optionally) A linux computer connected to a kinect publishing a pointcloud

Currently in the armlab you will need the computer `armor`. You will also need the computer `realtime` and `loki` if you are running the physical robot.

### On the windows computer (Armor) 
1. Connect HTC Vive to computer (or plug in battery if using wireless)
2. (If using wireless) Launch VIVE Wireless
3. Launch SteamVR
   1. Launch Steam
   2. Click on Library > VR
   3. Select SteamVR and click play
   
   If you are running SteamVR for first time, please start the SteamVR room setup to calibrate the device.


### On the linux computer running ROS (The linux VM inside Armor)
Note in this example the computer `loki` is connected to the Kinect. `realtime` provides the interface to the physical robot.

Terminal 1:
```
ssh loki
roscore
```

Terminal 2:
```
ssh realtime
export ROS_MASTER_URI=http://loki.local:11311
roslaunch victor_hardware_interface dual_arm_lcm_bridge.launch
```

Terminal 3:
```
ssh loki
roslaunch mps_launch_files kinect_vicon_real_robot.launch pov:="victor_head" fps_limit:="10"
```

Terminal 4:
```
export ROS_MASTER_URI=http://loki.local:11311
rosrun dual_arm_teleop make_victor_fast.py
roslaunch unity_launch_files unity_setup.launch
```
Note: Current version mismatch. Need to edit `victor_utils`. Delete the `/` from the service call name. Then `export ROS_NAMESPACE=/victor`

(Note, if `make_victor_fast` fails, check that the Victor robot is in impedance mode)

### Back on the Windows computer (Armor)
1. Launch unity, and open the project `unity_victor_teleop`
2. Check that the IP is correct. `RosBridgeSettings` from the unity panel must match the IP from the linux computer running the unity setup (The IP of the vm on armor, which you can see using `ifconfig`). The form is `ws://10.10.10.124:9090`
3. `Play` the unity environment



## Controls
![alt-text][vive-controller-layout]

The controls for operating the robot are as follows:
1. **Menu button** - 
2. **Trackpad**  
   Teleportation
3. **System button**
4. **Status light**
5. **Micro-USB port**
6. **Tracking sensor**
7. **Trigger** - open and close the grippers
8. **Grip button** - "Grab" the robot arm


## Troubleshooting
* If SteamVR is having trouble detecting the Vive headset, try switching on/off the SteamVR beta

   Found by right clicking SteamVR > Properties > Betas >  Select the beta you would like to opt into
* Read the SteamVR for Linux guide to make sure everything is configured properly

   [SteamVR for Linux guide](https://github.com/ValveSoftware/SteamVR-for-Linux)

* If Unity is not detecting any input from ROS (kinematics, kinect), check the ip address match between the Unity RosConnector node and the ubuntu VM (run `ifconfig`)


[vive-controller-layout]: https://www.vive.com/media/filer_public/17/5d/175d4252-dde3-49a2-aa86-c0b05ab4d445/guid-2d5454b7-1225-449c-b5e5-50a5ea4184d6-web.png "Vive Controller Layout"

* If errors about missing packages, make sure you installed everything needed for ROS-sharp. e.g. `cannot find rosbridge_websocket` means you missed installing `sudo apt-get install ros-kinetic-rosbridge-server`
