#!/usr/bin/env bash
systemctl stop deepracer-core
source /opt/ros/foxy/setup.bash 
source /opt/intel/openvino_2021/bin/setupvars.sh
source ./install/setup.bash