#!/bin/bash
# Run motor_driver_node with EXPERIMENTAL directional safety enabled.
# Survives terminal close; intended for manual SLAM mapping sessions.
#
# Usage: run_motor_directional.sh [initial_odom_x] [initial_odom_y] [initial_odom_yaw]
# Capture the current pose BEFORE killing the previous motor node:
#   ros2 topic echo /control/odom --once --field pose.pose
# and pass it here so the odom frame stays continuous for SLAM.

INIT_X="${1:-0.0}"
INIT_Y="${2:-0.0}"
INIT_YAW="${3:-0.0}"

source /opt/ros/humble/setup.bash
source /home/hprobot/amr_ws/install/setup.bash

exec ros2 run helper_control motor_driver --ros-args \
  -r __node:=motor_driver_node \
  -p serial_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 \
  -p cmd_vel_topic:=/control/cmd_vel_test \
  -p directional_safety_enabled:=true \
  -p directional_scan_topic:=/perception/scan/raw \
  -p initial_odom_x:="$INIT_X" \
  -p initial_odom_y:="$INIT_Y" \
  -p initial_odom_yaw:="$INIT_YAW"
