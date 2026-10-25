#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export PYTHONPATH="/opt/qda-brain-python:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${QDA_ROS_DOMAIN_ID:-8}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///ws/docker/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
exec python3 -m qda_brain.mcp_server
