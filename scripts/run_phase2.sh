#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source ~/ishb_ws/marsim_ws/devel/setup.bash --extend
source ~/ishb_ws/fastlio_ws/devel/setup.bash --extend
source ~/ishb_ws/super_ws/devel/setup.bash --extend
source ~/ishb_ws/ishb_ws/devel/setup.bash --extend
roslaunch ishb_bringup phase2_system.launch lidar_type:=mid360
