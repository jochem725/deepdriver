#!/usr/bin/env bash
ros2 service call /traffic_navigation_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv  "{max_speed_pct: $1}"