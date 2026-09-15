"""Exercise callbacks without creating ROS nodes, serial ports, or publishers."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist
from rclpy.time import Time

from helper_control.directional_safety import StopLatch
from helper_control.keyboard_teleop_node import KeyboardTeleopNode
from helper_control.kinematics_engine import KinematicsEngine
from helper_control.motor_driver_node import MotorDriverNode
from helper_control.robot_parameters import RobotParameters


def motor():
    now = Time(seconds=10)
    node = SimpleNamespace(
        directional_enabled=True,
        directional_scan=SimpleNamespace(check=Mock(return_value=(True, 'clear'))),
        directional_stamp=Time(seconds=9.9),
        directional_cache={}, directional_latch=StopLatch(),
        directional_reason=None, safety_stop_active=False,
        current_left_rpm=0, current_right_rpm=0,
        kinematics=KinematicsEngine(RobotParameters()),
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        get_logger=lambda: Mock(),
    )
    return node


@pytest.mark.parametrize('stamp', [None, Time(seconds=9), Time(seconds=11)])
def test_missing_stale_future_scan_stops_and_latches(stamp):
    node = motor()
    node.directional_stamp = stamp
    assert MotorDriverNode.apply_directional_gate(node, 100, 100) == (0, 0)
    assert node.safety_stop_active
    assert node.directional_latch.blocked


def test_clear_obstacle_requires_zero_then_new_command():
    node = motor()
    node.directional_scan.check.return_value = (False, 'obstacle_in_path')
    assert MotorDriverNode.apply_directional_gate(node, 100, 100) == (0, 0)
    node.directional_cache.clear()
    node.directional_scan.check.return_value = (True, 'clear')
    assert MotorDriverNode.apply_directional_gate(node, 100, 100) == (0, 0)
    MotorDriverNode.cmd_vel_callback(node, Twist())
    command = Twist()
    command.linear.x = 0.03
    MotorDriverNode.cmd_vel_callback(node, command)
    assert MotorDriverNode.apply_directional_gate(node, 100, 100) == (100, 100)


def test_zero_target_does_not_clear_latch_by_itself():
    node = motor()
    node.directional_latch.blocked = True
    assert MotorDriverNode.apply_directional_gate(node, 0, 0) == (0, 0)
    assert node.directional_latch.blocked
    assert node.safety_stop_active  # Immediate brake, not a slow RPM ramp.


def test_motor_input_is_speed_capped():
    node = motor()
    command = Twist()
    command.linear.x = 10.0
    command.angular.z = -10.0
    MotorDriverNode.cmd_vel_callback(node, command)
    v, w = node.kinematics.forward_kinematics(
        node.target_left_rpm, node.target_right_rpm)
    assert abs(v - 0.375) < 0.001
    assert abs(w + 1.5) < 0.002


def test_unsafe_current_ramp_direction_also_stops():
    node = motor()
    node.current_left_rpm = node.current_right_rpm = -100
    node.directional_scan.check.side_effect = [(True, 'clear'), (False, 'obstacle_in_path')]
    assert MotorDriverNode.apply_directional_gate(node, 100, 100) == (0, 0)


def test_legacy_stop_still_works_when_new_mode_disabled():
    node = motor()
    node.directional_enabled = False
    node.safety_stop_enabled = True
    node.stop_on_unknown = True
    node.get_obstacle_state = lambda: (True, 'obstacle', 0.2, '/obstacle')
    assert MotorDriverNode.apply_safety_gate(node, 100, 100) == (0, 0)


def test_150ms_first_repeat_keeps_command_and_release_still_stops():
    now = [10.0]
    publisher = Mock()
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])),
        publisher=publisher, key_timeout=0.25, last_key_time=Time(seconds=10),
        last_linear=0.05, last_angular=0.0, stopped=False,
        speed=0.05, turn=0.2,
    )
    now[0] = 10.14
    KeyboardTeleopNode.timer_callback(node)
    assert publisher.publish.call_args.args[0].linear.x == 0.05
    now[0] = 10.15
    KeyboardTeleopNode.handle_key(node, 'i')
    now[0] = 10.30
    KeyboardTeleopNode.timer_callback(node)
    assert publisher.publish.call_args.args[0].linear.x == 0.05
    now[0] = 10.41
    KeyboardTeleopNode.timer_callback(node)
    assert publisher.publish.call_args.args[0].linear.x == 0.0
    assert node.stopped


def test_key_timeout_publishes_one_zero_command():
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)),
        last_key_time=Time(seconds=9), key_timeout=0.25,
        last_linear=0.05, last_angular=0.2, stopped=False, publisher=Mock())
    KeyboardTeleopNode.timer_callback(node)
    node.publisher.publish.assert_called_once()
    output = node.publisher.publish.call_args.args[0]
    assert output.linear.x == output.angular.z == 0
    KeyboardTeleopNode.timer_callback(node)
    node.publisher.publish.assert_called_once()
