# Guide of the Universal Robot (eg: UR5)
| **`Ubuntu 1804 & ROS Melodic Morenia`** |  


till now, we still used the deprecated `ur_modern_driver` from [here](https://github.com/ros-industrial/ur_modern_driver).

We would follow the latest `Universal_Robots_ROS_Driver` at [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) in the future. 


## 1. Installation

``` bash
$ mkdir ur_ws && cd ur_ws

# 1. clone the ros-industrial/universal_robot
$ git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot


# 2. clone the ur_modern_driver 
$ git clone https://github.com/ros-industrial/ur_modern_driver.git
```

There are bugs in `ur_modern_driver` (issue [#58](https://github.com/ros-industrial/ur_modern_driver/issues/58#issuecomment-240197829)), fix it in file `ur_modern_driver/src/ur_hardware_interface.cpp`: 

```
-       controller_it->hardware_interface
￼+       controller_it->type

-       stop_controller_it->hardware_interface
￼+       stop_controller_it->type

```  

```bash
# 3. checking dependencies
$ rosdep update
$ rosdep install --rosdistro melodic --ignore-src --from-paths src

# 4. building
$ catkin_make

# 5. activate this workspace (change bash to zsh if you use zsh)
$ source devel/setup.bash

```


## 2. Configure IP 

1. plug the ethernet cabel in your PC
2. on the UR control panels, press *Setup Robots* -> *NetWork* -> *Static Address*.  
3. configure the IP address, subnet mask, gate way, preferred DNS server. eg:   
IP address : 192.168.1.102  
Subnet mask : 255.255.255.0  
Default gateway : 192.168.1.1   
Prefferred DNS server : 192.168.1.1  
Alternative DNS server : 0.0.0.0  

4. on your PC, go to *Setting* -> *Network* -> *Wired* -> *IPv4*.  use the *Manual IPv4* and set your PC's address, netmask and gateway.  make sure the ip address of your PC is not in conflict with UR.  

## 3. Bringup UR

At your UR control panel, go to the initialization screen, press *ON* -> *START*.  
At your PC,  `cd ur_ws` and  `source devel/setup.bash` at each terminal. 

Start UR driver
```bash
$1 roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.102
```

Start moveit:
```bash
$2 roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```

Start rviz to visualize
```bash
$3 roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

