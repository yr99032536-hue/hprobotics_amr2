"""Offline checks: never connect to a motor or publish movement commands."""

import math
import json
from pathlib import Path

import pytest

from helper_control.directional_safety import DirectionalSafety, StopLatch


def scan(points=(), invalid=None, pose=(0.149, 0.0, math.pi)):
    ranges = [5.0] * 721
    sx, sy, yaw = pose
    for x, y in points:
        angle = (math.atan2(y - sy, x - sx) - yaw + math.pi) % (
            2 * math.pi) - math.pi
        index = round((angle + math.pi) / (math.pi / 360))
        ranges[index] = math.hypot(x - sx, y - sy)
    if invalid is not None:
        ranges = [invalid] * 721
    return DirectionalSafety(ranges, -math.pi, math.pi / 360, 0.01, 8.0, pose)


def test_clear_front_allows_forward():
    assert scan().check(0.05, 0) == (True, 'clear')


def test_front_obstacle_blocks_forward():
    assert scan([(0.25, 0.0)]).check(0.05, 0)[0] is False


def test_rear_obstacle_does_not_block_forward():
    assert scan([(-0.50, 0.0)]).check(0.05, 0)[0] is True


def test_rear_sensor_can_escape_front_obstacle():
    assert scan([(0.25, 0.0)], pose=(-0.45, 0, 0)).check(-0.05, 0)[0] is True


@pytest.mark.parametrize('command', [(-0.05, 0), (0, 0.2), (0, -0.2)])
def test_exposed_scanner_has_no_artificial_body_shadow(command):
    assert scan().check(*command) == (True, 'clear')


@pytest.mark.parametrize('invalid', [math.inf, math.nan, 0.0, -1.0, 9.0])
def test_invalid_ranges_are_not_clear(invalid):
    assert scan(invalid=invalid).check(0.05, 0)[0] is False


@pytest.mark.parametrize('command', [(0, 0.2), (0, -0.2), (0.05, 0.2), (-0.05, -0.2)])
def test_rotation_and_arcs_block_near_side_obstacle(command):
    # Points on both swept sides; the previous test accidentally depended on
    # the fictitious body-shadow stop for clockwise motion.
    assert scan([(0.20, 0.24), (0.20, -0.24),
                 (-0.39, 0.24), (-0.39, -0.24)]).check(*command)[0] is False


def test_zero_is_always_safe():
    assert scan(invalid=math.nan).check(0, 0)[0] is True


@pytest.mark.parametrize('command', [(1, 0), (0, 2), (math.nan, 0), (0, math.inf)])
def test_invalid_or_fast_command_is_rejected(command):
    assert scan().check(*command)[0] is False


def test_held_key_does_not_restart_when_obstacle_clears():
    latch = StopLatch()
    assert latch.evaluate(False) is False
    latch.command(0.05, 0)
    assert latch.evaluate(True) is False
    latch.command(0, 0)
    assert latch.evaluate(True) is True


def test_bad_scan_metadata_is_rejected():
    with pytest.raises(ValueError):
        DirectionalSafety([1.0], 0, 0, 0.1, 8, (0, 0, 0))


@pytest.mark.parametrize('value', [math.inf, math.nan, 0.0, -1.0, 9.0])
def test_sparse_missing_returns_do_not_invent_obstacles(value):
    ranges = [5.0 if i % 6 else value for i in range(721)]
    sensor = DirectionalSafety(ranges, -math.pi, math.pi / 360,
                               0.01, 8.0, (0.149, 0, math.pi))
    for command in [(0.05, 0), (-0.05, 0), (0, 0.2), (0, -0.2)]:
        assert sensor.check(*command) == (True, 'clear')


@pytest.mark.parametrize('seam', [False, True])
def test_large_missing_sector_alone_does_not_stop(seam):
    ranges = [5.0] * 721
    if seam:
        ranges[:15] = [math.inf] * 15
        ranges[-15:] = [math.inf] * 15
    else:
        ranges[345:375] = [math.inf] * 30
    sensor = DirectionalSafety(ranges, -math.pi, math.pi / 360,
                               0.01, 8.0, (0.149, 0, 0))
    assert sensor.check(0.05, 0) == (True, 'clear')


def test_partial_field_of_view_is_not_accepted_as_360_coverage():
    sensor = DirectionalSafety([5.0] * 181, 0, math.pi / 360,
                               0.01, 8, (0.149, 0, 0))
    assert sensor.check(0.05, 0) == (False, 'scan_coverage_insufficient')


def test_low_valid_fraction_alone_does_not_stop():
    ranges = [5.0 if i % 4 == 0 else math.inf for i in range(721)]
    sensor = DirectionalSafety(ranges, -math.pi, math.pi / 360,
                               0.01, 8, (0.149, 0, 0))
    assert sensor.check(0.05, 0) == (True, 'clear')


def test_single_near_point_is_not_removed_as_noise_or_chassis():
    sensor = scan([(0.185, 0.035)])
    assert sensor.check(0.05, 0) == (False, 'obstacle_in_path')
    assert sensor.check(-0.05, 0) == (True, 'clear')


def test_front_camera_protrusion_is_protected():
    assert scan([(0.36, 0)]).check(0.05, 0) == (False, 'obstacle_in_path')


def test_six_cm_margin_allows_old_buffer_but_stops_inside_new_buffer():
    assert DirectionalSafety.margin == 0.06
    # After 0.10m translation: gaps to the 0.215m front edge are 8.5/4.5cm.
    assert scan([(0.40, 0)]).check(0.05, 0) == (True, 'clear')
    assert scan([(0.36, 0)]).check(0.05, 0) == (False, 'obstacle_in_path')


@pytest.mark.parametrize('v,w', [(0.375,0),(-0.375,0),(0,1.5),(0,-1.5),(.375,1.5)])
def test_new_speed_caps_allow_clear_space(v,w):
    assert scan().check(v,w) == (True, 'clear')


@pytest.mark.parametrize('v,w', [(0.38,0),(0,1.51)])
def test_above_new_speed_caps_rejected(v,w):
    assert scan().check(v,w) == (False, 'speed_limit')


def test_faster_forward_sweep_checks_farther_obstacle():
    sensor = scan([(0.51,0)])
    assert sensor.check(.05,0)[0]
    assert sensor.check(.125,0) == (False, 'obstacle_in_path')


def test_threefold_cap_checks_obstacle_beyond_old_collection_radius():
    sensor = scan([(1.0, 0)])
    assert sensor.check(.125,0) == (True, 'clear')
    assert sensor.check(.375,0) == (False, 'obstacle_in_path')


def test_legacy_filter_is_explicit_and_excludes_rear_sector():
    original = scan([(-0.11, -0.003), (-0.134, 0.014)])
    legacy = DirectionalSafety(original.ranges, original.angle_min,
                               original.increment, original.range_min,
                               original.range_max, (0.149, 0, math.pi),
                               legacy_scan_filter=True)
    assert not original.check(0, 0.2)[0]
    assert not original.check(0, -0.2)[0]
    assert legacy.check(0, 0.2) == (True, 'clear')
    assert legacy.check(0, -0.2) == (True, 'clear')
    assert legacy.healthy


def test_legacy_filter_keeps_front_obstacle():
    original = scan([(0.36, 0)])
    legacy = DirectionalSafety(original.ranges, original.angle_min,
                               original.increment, .01, 8, (0.149, 0, math.pi),
                               legacy_scan_filter=True)
    assert legacy.check(.05, 0) == (False, 'obstacle_in_path')


def test_legacy_filter_does_not_hide_raw_sensor_failure():
    legacy = DirectionalSafety([math.inf]*721, -math.pi, math.pi/360,
                               .01, 8, (0.149, 0, math.pi),
                               legacy_scan_filter=True)
    assert legacy.check(0, .2) == (False, 'scan_coverage_insufficient')


def test_captured_scan_does_not_fail_for_invented_body_shadow():
    path = Path(__file__).parent / 'fixtures' / 'directional_scan_20260914.json'
    for frame in json.loads(path.read_text())['frames']:
        sensor = DirectionalSafety(
            [float(v) if v != 'invalid' else math.nan for v in frame['ranges']],
            frame['angle_min'], frame['angle_increment'],
            frame['range_min'], frame['range_max'], (0.149, 0, math.pi))
        assert sensor.healthy
        for command in [(0.05, 0), (-0.05, 0), (0, 0.2), (0, -0.2)]:
            allowed, reason = sensor.check(*command)
            assert reason in ('clear', 'obstacle_in_path')
        # A single real return must still stop the requested path.
        sensor.points.append((0.35, 0.0))
        assert sensor.check(0.05, 0) == (False, 'obstacle_in_path')
