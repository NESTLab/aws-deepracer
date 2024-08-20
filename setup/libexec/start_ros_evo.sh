#!/usr/bin/env bash

# This file is generally not intended to be run manually, but called by the start-ros-evo.service file

source /opt/ros/foxy/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
source ${HOME}/deepracer_nav2_ws/aws-deepracer/install/setup.bash

ros2 launch deepracer_bringup deepracer_evo_bare.launch.py