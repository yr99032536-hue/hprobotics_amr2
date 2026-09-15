"""Finite-point swept-footprint checks; no ROS or motor side effects.

This is an experimental driving aid, not a safety-rated emergency stop.
Uses the finite-return model documented by Nav2 Collision Monitor, not its node.
Missing rays are neither obstacles nor certified free space. Whole-scan health
checks reject wholly unusable scans. Requires an exposed 360-degree scanner.
"""

import math


class DirectionalSafety:
    """Check a low-speed command against a raw scan in base coordinates."""

    bounds = (-0.40, 0.215, -0.23, 0.23)  # Include the URDF camera protrusion.
    margin = 0.06  # Trial buffer; actual stopping distance still needs validation.
    horizon = 2.0
    max_linear = 0.375
    max_angular = 1.5

    def __init__(self, ranges, angle_min, angle_increment, range_min,
                 range_max, sensor_pose, legacy_scan_filter=False):
        self.ranges = tuple(ranges)
        self.angle_min = angle_min
        self.increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.sx, self.sy, self.yaw = sensor_pose
        metadata = (angle_min, angle_increment, range_min, range_max,
                    *sensor_pose)
        if (not all(math.isfinite(x) for x in metadata)
                or not 0 < angle_increment <= math.radians(1) or not ranges
                or range_min < 0 or range_max <= range_min
                or angle_increment * (len(ranges) - 1) > 2 * math.pi + 0.1):
            raise ValueError('invalid scan metadata')
        self.points = []
        xmin, xmax, ymin, ymax = self.bounds
        sweep_radius = (math.hypot(max(abs(xmin), abs(xmax)) + self.margin,
                                   max(abs(ymin), abs(ymax)) + self.margin)
                        + self.max_linear * self.horizon + 0.01)
        for i, distance in enumerate(ranges):
            if math.isfinite(distance) and range_min <= distance <= range_max:
                sensor_angle = angle_min + i * angle_increment
                # Explicit compatibility with front_lidar_slam.launch.py.
                # This is a blind sector, NOT confirmed self-return removal.
                if legacy_scan_filter and (
                        -math.radians(40) < sensor_angle < math.radians(40)
                        or not 0.15 <= distance <= 8.0):
                    continue
                angle = sensor_angle + self.yaw
                x = self.sx + distance * math.cos(angle)
                y = self.sy + distance * math.sin(angle)
                # Bound by footprint + maximum translation, not a fixed radius.
                if math.hypot(x, y) <= sweep_radius:
                    self.points.append((x, y))
        # Health is checked on RAW data, before intentional exclusions.
        valid = [math.isfinite(r) and range_min <= r <= range_max for r in ranges]
        self.valid_fraction = sum(valid) / len(valid)
        span = angle_increment * (len(ranges) - 1)
        # Merge duplicate +/-pi endpoints for circular dropout measurement.
        if abs(span - 2 * math.pi) < 1e-3:
            valid[0] = valid[0] or valid[-1]
            valid.pop()
        run = longest = 0
        for usable in valid + valid:
            run = 0 if usable else run + 1
            longest = max(longest, run)
        self.missing_angle = min(longest, len(valid)) * angle_increment
        # Missing returns are not a reliable sensor-disconnection indicator.
        # Keep gap/fraction as diagnostics only. Reject completely unusable
        # scans and incompatible FOV; arrival age/TF are gated by the motor.
        self.healthy = (span >= 2 * math.pi - 2 * angle_increment
                        and any(valid))

    @classmethod
    def clearance(cls, x, y):
        """Signed distance to the physical, axis-aligned footprint."""
        xmin, xmax, ymin, ymax = cls.bounds
        dx = max(xmin - x, 0.0, x - xmax)
        dy = max(ymin - y, 0.0, y - ymax)
        if dx or dy:
            return math.hypot(dx, dy)
        return -min(x - xmin, xmax - x, y - ymin, ymax - y)

    def check(self, linear, angular):
        """Return (allowed, reason), checking combined translation/rotation."""
        if not all(math.isfinite(x) for x in (linear, angular)):
            return False, 'invalid_command'
        # Small allowance for motor RPM quantization after inverse kinematics.
        if abs(linear) > self.max_linear + 0.001 or abs(angular) > self.max_angular + 0.002:
            return False, 'speed_limit'
        if abs(linear) < 1e-9 and abs(angular) < 1e-9:
            return True, 'zero_command'
        if not self.healthy:
            return False, 'scan_coverage_insufficient'
        # Sweep steps limit corner displacement to under 1 cm at capped speed.
        steps = max(1, math.ceil(self.horizon *
                                (abs(linear) + 0.5 * abs(angular)) / 0.01))
        for n in range(1, steps + 1):
            t = self.horizon * n / steps
            theta = angular * t
            c, s = math.cos(theta), math.sin(theta)
            if abs(angular) < 1e-9:
                tx, ty = linear * t, 0.0
            else:
                tx = linear * s / angular
                ty = linear * (1 - c) / angular
            for x, y in self.points:
                # A bounding rectangle is NOT a calibrated self-return mask.
                # Keep even single points: thin obstacles must not be erased.
                initial = self.clearance(x, y)
                future = self.clearance(c * (x - tx) + s * (y - ty),
                                        -s * (x - tx) + c * (y - ty))
                if future < self.margin and future < initial - 1e-6:
                    return False, 'obstacle_in_path'
        return True, 'clear'


class StopLatch:
    """Prevent a held movement key from restarting after an obstruction clears."""

    def __init__(self):
        self.blocked = False

    def command(self, linear, angular):
        if linear == 0.0 and angular == 0.0:
            self.blocked = False

    def evaluate(self, allowed):
        if not allowed:
            self.blocked = True
        return allowed and not self.blocked
