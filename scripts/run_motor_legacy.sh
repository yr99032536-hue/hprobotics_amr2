#!/bin/bash
# Motor node in LEGACY scalar-gate mode (proven working 2026-09-15 morning).
# Directional mode blocked all motion: lidar publishes inconsistent
# 129-beam/0.5deg scans (64deg coverage claimed as 360deg) -> gate flaps
# between obstacle_in_path (self-returns) and coverage failure, latch holds.
source /opt/ros/humble/setup.bash
source /home/hprobot/amr_ws/install/setup.bash
exec ros2 run helper_control motor_driver --ros-args \
  -r __node:=motor_driver_node \
  -p serial_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 \
  -p cmd_vel_topic:=/control/cmd_vel_test \
  -p directional_safety_enabled:=false \
  -p initial_odom_x:="${1:-0.0}" -p initial_odom_y:="${2:-0.0}" -p initial_odom_yaw:="${3:-0.0}"
