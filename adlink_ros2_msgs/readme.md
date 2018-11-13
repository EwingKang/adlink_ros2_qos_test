# ADLINK ROS 2.0 Custom Messages  
    
## Developers    
* HaoChih, LIN (haochih.lin@adlinktech.com)  

## License    
Apache 2.0 (Copyright 2018 ADLINK Technology, Inc.)  

## Compile    
$ cd ROS2_WS  
$ ament build --isolated --symlink-install --parallel --only adlink_msgs  

## Verify  
$ ros2 msg list  
$ ros2 msg show MSG_NAME  

## ROS 2to1 Bridge Compile
According to ros_bridge issue 77:  
https://github.com/ros2/ros1_bridge/issues/77  

The way the dynamic_bridge is designed you should not need to specify the mapping in a yaml file if the massage package + name is the same between ros1 and ros2. You should also not need to change anything in the CMakLists or the package.xml of the bridge.  
  
From the error message it looks like your ROS1 workspace with your custom messages was not sourced when you built the bridge hence the (ROS 1 type '' message)  
Could you give more details about your setup (what version of the code you are using mostly and maybe also the platform and the rmw implementation you are using).  

Usually when I use custom messages I have:  
(in our case, custom msg pkg are: adlink_ros_msgs & adlink_ros2_msgs)  
* a ros2 workspace with just the core of ROS2  
* ros1 installed from binaries  
* an overlay catkin workspace with my custom message package  
* an overlay ament (ROS2) workspace with my custom message package  
  
The steps I use are:  
1. in one shell build my ros2 workspace (same as binary installed)  
2. in another shell build my catkin workspace  
3. in another shell: source my ros2 workspace and build my overlay workspace  
4. finally build the bridge in a new shell:  
 * source /opt/ros//setup.bash  
 * source <MY_ROS2_WORKSPACE>/install/(local_)setup.bash  
 * source my catkin workspace  
 * source my ros2 overlay workspace  
 * rm -rf ros2_ws/build/ros1_bridge  
 * build the bridge: $ ament build --only ros1_bridge  
 * OR throught isolated building way: $ ament build --isolated --build-tests --symlink-install --only ros1_bridge --force-cmake-configure  
 

## Trouble Shooting
1. Remember to clone the "ardent" branch of "ros_bridge" pkg instead of "master" branch.
