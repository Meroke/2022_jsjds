## Introduction

***Before starting, please reader have basic ROS and navigation knowledge, or experienced automatic driver.***

This repositority is *Third Generation*  since the fisrt race in Nanjing China Computer Design Competition.

Thanks to seniors, *Huang Yuyang, He Niaofeng, Hong Kai, Dai Haojun, Yu Zhengni*, who created the glorious history of NBUT Elite Lab.

**Difference:**

In the 3th race, the rules of the game have been abandoned second camera to detect boxes with province labels in the gound. It also reduces the number of carries that can be carried at a time.

In total, it simplifies the ruels and reinforces the competitiveness. So only speed and precision are included in the competitive factors.

  **Attention**: *The repositority for 2022 JSJSJ Competition. The running platform is EAIBot. Also another version of Mecanum Car is available in other repository.*

## Document Architecture


```
├── README.md  
├── slam_version.txt
└── src
    ├── CMakeLists.txt
    ├── dashgo                               # main functional node here
    │   ├── dashgo_description  # car model
    │   ├── dashgo_nav                # navigation launch file.  Main Entrance.
    │   ├── dashgo_rviz				
    │   ├── dashgo_tools
    │   ├── get_odom     			 # get distance from  the back of 360 degree lidar. (Not suit EAIBot)
    │   ├── laser_filters
    │   ├── mapping
    │   ├── README.md
    │   ├── ReadMe.txt
    │   ├── rplidar_ros				# rplidar node (also blew is ydlidar node. Select one deponds on the brand of lidar.)
    │   ├── rviz
    │   ├── single_scan				# get distance from  a laser module in the  back of the car. 
    │   ├── smart_node			   #  include IMU config and velocity_smoother
    │   ├── traffic
    │   └── ydlidar_ros-master  # ydlidar node (Suit EAIBot).
    ├── dashgo_src					 #  Main packages of ROS environment 
    │   ├── amcl
    │   ├── base_local_planner
    │   ├── carrot_planner
    │   ├── clear_costmap_recovery
    │   ├── costmap_2d
    │   ├── dwa_local_planner 
    │   ├── fake_localization
    │   ├── global_planner
    │   ├── map_server
    │   ├── move_base
    │   ├── move_slow_and_clear
    │   ├── nav_core
    │   ├── navfn
    │   ├── navigation
    │   ├── ReadMe.txt

​    │   ├── robot_pose_ekf
​    │   ├── rotate_recovery
​    │   ├── teb_local_planner
​    │   └── voxel_grid
​    └── yocs_velocity_smoother


42 directories, 10 files
```


## Remaining Questions

**Constant Jerk**

In order to pursue velocity, we increase the acceleration and speed parameters, likemax_vel_x, acc_lim_x etc. 

To accommodate such changes, we should add acceleration limiting parameter's value when approaching the target location to avoid collision, like penalty_epsilon.  After setting as above, the robot obtains abilities of high speed and sudden braking. 

But this leads to a problem that chassis motion planning will occasionally suck in constant jerk, which may be because of acceleration changes with high frequency and insufficient CPU performance.

**Lidar Drift**

When launching the lidar and rviz, the lidar laser is matched with map. As the robot  moves, the laser deviated from the map, especially when the robot rotates.

Fortunately, we found this casued by driving wheel skid. Because of the rules, we can't change the wheel. Therefore, we put double-sided tape on the wheels and get remarkable results.

## Optimized Direction

**Path Planning**

GLoabal path planning 

-  A*  
-  JPS

Local path planning 

- TEB