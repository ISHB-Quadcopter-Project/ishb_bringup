#!/usr/bin/env bash
# source_env.sh — source the full workspace chain, no launch.
# Use before catkin_make, or in any new terminal that needs the workspaces.
source /opt/ros/noetic/setup.bash
source ~/ishb_ws/marsim_ws/devel/setup.bash --extend
source ~/ishb_ws/fastlio_ws/devel/setup.bash --extend
source ~/ishb_ws/super_ws/devel/setup.bash --extend
source ~/ishb_ws/vslam_ws/devel/setup.bash --extend
