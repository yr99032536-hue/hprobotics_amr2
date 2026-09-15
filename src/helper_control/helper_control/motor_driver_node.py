import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from helper_msgs.msg import ObstacleDecision
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from helper_control.directional_safety import DirectionalSafety, StopLatch
from helper_control.kinematics_engine import KinematicsEngine
from helper_control.md200t_driver import MD200TDriver
from helper_control.robot_parameters import RobotParameters


PID_STOP_STATUS = 24
STOP_BRAKE = 2
PID_COMMAND = 10
CMD_BRAKE = 4


class MotorDriverNode(Node):
    """ROS 2 bridge from Twist commands to the MD200T motor driver."""

    def __init__(self):
        super().__init__('motor_driver_node')

        self.cfg = RobotParameters()
        self._declare_parameters()
        self._load_parameters()
        initial_pose = tuple(float(self.get_parameter(name).value) for name in (
            'initial_odom_x', 'initial_odom_y', 'initial_odom_yaw'))
        if not all(math.isfinite(value) for value in initial_pose):
            raise ValueError('Initial odometry pose must be finite')

        self.kinematics = KinematicsEngine(self.cfg)

        self.driver = None
        self.connected = False
        if self.dry_run:
            self.get_logger().warn(
                'dry_run=true: MD200T serial output is disabled.'
            )
        else:
            self.driver = MD200TDriver(
                port=self.cfg.SERIAL_PORT,
                baudrate=self.cfg.BAUD_RATE,
                robot_id=self.cfg.DRIVER_ID,
                max_rpm=self.cfg.MAX_RPM,
                com_watch_delay=self.cfg.COM_WATCH_DELAY,
            )
            self.connected = self.driver.connect()
            if self.connected:
                self.connected = self.driver.initialize_motor()
            if self.connected:
                self._configure_stop_brake()

        if self.connected:
            self.get_logger().info(
                'MD200T connected on '
                f'{self.cfg.SERIAL_PORT} at {self.cfg.BAUD_RATE} bps'
            )
        elif not self.dry_run:
            self.get_logger().error(
                'MD200T is not connected. '
                f'port={self.cfg.SERIAL_PORT}, '
                f'error={self.driver.last_error or "unknown"}. '
                'RPM commands will not be sent.'
            )

        self.target_left_rpm = 0
        self.target_right_rpm = 0
        self.current_left_rpm = 0
        self.current_right_rpm = 0
        self.actual_left_rpm = 0
        self.actual_right_rpm = 0
        self.last_feedback_time = None
        self.last_feedback_log_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()
        self.pending_brake_timer = None
        self.is_stopped = True
        self.safety_stop_active = False
        self.obstacle_decision = 'unknown'
        self.obstacle_distance = math.inf
        self.last_obstacle_time = None
        self.obstacle_states = {}
        self.directional_enabled = self.get_parameter('directional_safety_enabled').value
        self.directional_scan = None
        self.directional_stamp = None
        self.directional_cache = {}
        self.directional_latch = StopLatch()
        self.directional_reason = None
        if self.directional_enabled:
            if self.get_parameter('directional_legacy_scan_filter').value:
                self.get_logger().warn(
                    'LEGACY SCAN FILTER ACTIVE: sensor -40..40 deg and ranges '
                    'outside 0.15..8m excluded. Rear blind sector; NOT verified '
                    'self filtering. Supervised low-speed operation only.')
            self.scan_tf_buffer = Buffer()
            self.scan_tf_listener = TransformListener(self.scan_tf_buffer, self)
            self.scan_sub = self.create_subscription(
                LaserScan, self.get_parameter('directional_scan_topic').value,
                self.directional_scan_callback, qos_profile_sensor_data)
            self.get_logger().warn(
                'EXPERIMENTAL directional safety: raw scan replaces scalar '
                f'obstacle decisions; max {DirectionalSafety.max_linear} m/s, '
                f'{DirectionalSafety.max_angular} rad/s. '
                'Physical emergency stop is independent. Validate footprint first.')
            self.get_logger().info(
                'Directional finite-point sweep v2: '
                f'footprint={DirectionalSafety.bounds}, '
                f'margin={DirectionalSafety.margin:.3f}m, '
                'partial scan gaps are diagnostic only; '
                'all-invalid scan, FOV, freshness and TF checks retained')

        self.x, self.y, self.theta = initial_pose
        self.last_odom_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.obstacle_subs = []
        for topic in self.obstacle_topics:
            self.obstacle_subs.append(
                self.create_subscription(
                    ObstacleDecision,
                    topic,
                    lambda msg, topic=topic: self.obstacle_callback(
                        msg,
                        topic,
                    ),
                    10,
                )
            )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0),
            self.control_loop,
        )
        self.odom_timer = self.create_timer(
            1.0 / max(self.cfg.ODOM_PUBLISH_RATE, 1.0),
            self.publish_odom,
        )

        self.get_logger().info(f'subscribing to {self.cmd_vel_topic}')
        self.get_logger().info(
            f'safety obstacle topics: {", ".join(self.obstacle_topics)}'
        )
        self.get_logger().info(
            'motor mapping: motor1=left, motor2=right, '
            f'left_sign={self.cfg.LEFT_FORWARD_SIGN}, '
            f'right_sign={self.cfg.RIGHT_FORWARD_SIGN}, '
            f'swap_motors={self.cfg.SWAP_MOTORS}'
        )

    def _configure_stop_brake(self):
        self.driver.send_param(PID_STOP_STATUS, STOP_BRAKE, 1)
        self.get_logger().info('PID24 STOP_STATUS set to STOP_BRAKE(2).')

    def _declare_parameters(self):
        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel_safe')
        self.declare_parameter('odom_topic', '/control/odom')
        self.declare_parameter('obstacle_topic', '/perception/obstacle/fused')
        self.declare_parameter(
            'obstacle_topics',
            [
                '/perception/obstacle/fused',
                '/perception/obstacle/range',
                '/perception/obstacle/depth',
            ],
        )
        self.declare_parameter('serial_port', self.cfg.SERIAL_PORT)
        self.declare_parameter('baud_rate', self.cfg.BAUD_RATE)
        self.declare_parameter('driver_id', self.cfg.DRIVER_ID)
        self.declare_parameter('wheel_radius', self.cfg.WHEEL_RADIUS)
        self.declare_parameter('track_width', self.cfg.TRACK_WIDTH)
        self.declare_parameter('gear_ratio', self.cfg.GEAR_RATIO)
        self.declare_parameter('max_rpm', self.cfg.MAX_RPM)
        self.declare_parameter('max_linear_vel', self.cfg.MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel', self.cfg.MAX_ANGULAR_VEL)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_publish_rate', self.cfg.ODOM_PUBLISH_RATE)
        self.declare_parameter('cmd_timeout', self.cfg.CMD_TIMEOUT)
        self.declare_parameter('com_watch_delay', self.cfg.COM_WATCH_DELAY)
        self.declare_parameter('accel_rpm_per_sec', self.cfg.ACCEL_RPM_PER_SEC)
        self.declare_parameter('decel_rpm_per_sec', self.cfg.DECEL_RPM_PER_SEC)
        self.declare_parameter(
            'use_cmd_brake_on_stop',
            self.cfg.USE_CMD_BRAKE_ON_STOP,
        )
        self.declare_parameter('brake_delay_sec', self.cfg.BRAKE_DELAY_SEC)
        self.declare_parameter('feedback_enabled', self.cfg.FEEDBACK_ENABLED)
        self.declare_parameter(
            'feedback_read_timeout',
            self.cfg.FEEDBACK_READ_TIMEOUT,
        )
        self.declare_parameter('feedback_timeout', self.cfg.FEEDBACK_TIMEOUT)
        self.declare_parameter(
            'feedback_log_period',
            self.cfg.FEEDBACK_LOG_PERIOD,
        )
        self.declare_parameter('left_forward_sign', self.cfg.LEFT_FORWARD_SIGN)
        self.declare_parameter(
            'right_forward_sign',
            self.cfg.RIGHT_FORWARD_SIGN,
        )
        self.declare_parameter('left_rpm_scale', self.cfg.LEFT_RPM_SCALE)
        self.declare_parameter('right_rpm_scale', self.cfg.RIGHT_RPM_SCALE)
        self.declare_parameter('swap_motors', self.cfg.SWAP_MOTORS)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('invert_left_motor', False)
        self.declare_parameter('invert_right_motor', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('safety_stop_enabled', True)
        self.declare_parameter('stop_on_unknown', True)
        self.declare_parameter('obstacle_timeout', 1.0)
        self.declare_parameter('dry_run_log_period', 1.0)
        self.declare_parameter('directional_safety_enabled', False)
        self.declare_parameter('directional_scan_topic', '/perception/scan/raw')
        self.declare_parameter('directional_legacy_scan_filter', False)
        self.declare_parameter('initial_odom_x', 0.0)
        self.declare_parameter('initial_odom_y', 0.0)
        self.declare_parameter('initial_odom_yaw', 0.0)

    def _load_parameters(self):
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.obstacle_topics = self._unique_topics(
            [
                self.obstacle_topic,
                *self.get_parameter('obstacle_topics').value,
            ]
        )
        self.cfg.SERIAL_PORT = self.get_parameter('serial_port').value
        self.cfg.BAUD_RATE = int(self.get_parameter('baud_rate').value)
        self.cfg.DRIVER_ID = int(self.get_parameter('driver_id').value)
        self.cfg.WHEEL_RADIUS = float(
            self.get_parameter('wheel_radius').value
        )
        self.cfg.TRACK_WIDTH = float(
            self.get_parameter('track_width').value
        )
        self.cfg.GEAR_RATIO = float(
            self.get_parameter('gear_ratio').value
        )
        self.cfg.MAX_RPM = int(self.get_parameter('max_rpm').value)
        self.cfg.MAX_LINEAR_VEL = float(
            self.get_parameter('max_linear_vel').value
        )
        self.cfg.MAX_ANGULAR_VEL = float(
            self.get_parameter('max_angular_vel').value
        )
        self.cfg.ODOM_PUBLISH_RATE = float(
            self.get_parameter('odom_publish_rate').value
        )
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.cfg.CMD_TIMEOUT = self.cmd_timeout
        self.cfg.COM_WATCH_DELAY = int(
            self.get_parameter('com_watch_delay').value
        )
        self.cfg.ACCEL_RPM_PER_SEC = float(
            self.get_parameter('accel_rpm_per_sec').value
        )
        self.cfg.DECEL_RPM_PER_SEC = float(
            self.get_parameter('decel_rpm_per_sec').value
        )
        self.cfg.USE_CMD_BRAKE_ON_STOP = bool(
            self.get_parameter('use_cmd_brake_on_stop').value
        )
        self.cfg.BRAKE_DELAY_SEC = float(
            self.get_parameter('brake_delay_sec').value
        )
        self.feedback_enabled = bool(
            self.get_parameter('feedback_enabled').value
        )
        self.feedback_read_timeout = float(
            self.get_parameter('feedback_read_timeout').value
        )
        self.feedback_timeout = float(
            self.get_parameter('feedback_timeout').value
        )
        self.feedback_log_period = float(
            self.get_parameter('feedback_log_period').value
        )
        self.cfg.LEFT_FORWARD_SIGN = int(
            self.get_parameter('left_forward_sign').value
        )
        self.cfg.RIGHT_FORWARD_SIGN = int(
            self.get_parameter('right_forward_sign').value
        )
        self.cfg.LEFT_RPM_SCALE = float(
            self.get_parameter('left_rpm_scale').value
        )
        self.cfg.RIGHT_RPM_SCALE = float(
            self.get_parameter('right_rpm_scale').value
        )
        self.cfg.SWAP_MOTORS = bool(
            self.get_parameter('swap_motors').value
        )
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.invert_left_motor = bool(
            self.get_parameter('invert_left_motor').value
        )
        self.invert_right_motor = bool(
            self.get_parameter('invert_right_motor').value
        )
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.safety_stop_enabled = bool(
            self.get_parameter('safety_stop_enabled').value
        )
        self.stop_on_unknown = bool(
            self.get_parameter('stop_on_unknown').value
        )
        self.obstacle_timeout = float(
            self.get_parameter('obstacle_timeout').value
        )
        self.dry_run_log_period = float(
            self.get_parameter('dry_run_log_period').value
        )
        self.last_dry_run_log_time = self.get_clock().now()

        if self.invert_left_motor:
            self.cfg.LEFT_FORWARD_SIGN *= -1
        if self.invert_right_motor:
            self.cfg.RIGHT_FORWARD_SIGN *= -1

    def _unique_topics(self, topics):
        unique_topics = []
        for topic in topics:
            if not topic or topic in unique_topics:
                continue
            unique_topics.append(topic)
        return unique_topics

    def obstacle_callback(self, msg, topic=None):
        topic = topic or self.obstacle_topic
        self.obstacle_decision = msg.decision
        self.obstacle_distance = msg.distance
        self.last_obstacle_time = self.get_clock().now()
        self.obstacle_states[topic] = {
            'decision': msg.decision,
            'distance': msg.distance,
            'time': self.last_obstacle_time,
        }

    def directional_scan_callback(self, msg):
        self.directional_scan = None
        self.directional_stamp = None
        self.directional_cache.clear()
        try:
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0 <= age <= 0.3:
                return
            transform = self.scan_tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, stamp).transform
            q = transform.rotation
            # Only planar LiDAR mounting is supported by this checker.
            if abs(q.x) > 0.01 or abs(q.y) > 0.01:
                return
            yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
            self.directional_scan = DirectionalSafety(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max,
                (transform.translation.x, transform.translation.y, yaw),
                legacy_scan_filter=self.get_parameter(
                    'directional_legacy_scan_filter').value)
            self.directional_stamp = stamp
        except Exception as error:
            self.get_logger().warn(
                f'Directional scan/TF unavailable: {error}', throttle_duration_sec=2.0)

    def cmd_vel_callback(self, msg):
        linear, angular = msg.linear.x, msg.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.target_left_rpm = self.target_right_rpm = 0
            if self.directional_enabled:
                self.directional_latch.blocked = True
            return
        if self.directional_enabled:
            self.directional_latch.command(linear, angular)
            linear = max(-DirectionalSafety.max_linear,
                         min(DirectionalSafety.max_linear, linear))
            angular = max(-DirectionalSafety.max_angular,
                          min(DirectionalSafety.max_angular, angular))
        left_rpm, right_rpm = self.kinematics.inverse_kinematics(
            linear,
            angular,
        )

        self.target_left_rpm = left_rpm
        self.target_right_rpm = right_rpm
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0:
            dt = 1.0 / max(self.control_rate, 1.0)

        timed_out = self.watchdog_check_callback()

        left_rpm, right_rpm = self.apply_safety_gate(
            self.target_left_rpm,
            self.target_right_rpm,
        )

        hard_stop = timed_out or self.safety_stop_active
        if hard_stop:
            self.current_left_rpm = 0
            self.current_right_rpm = 0
            if not self.is_stopped:
                self.send_stop_command(
                    use_brake=timed_out or self.cfg.USE_CMD_BRAKE_ON_STOP
                )
            return

        self.current_left_rpm = self._ramp_rpm(
            self.current_left_rpm,
            left_rpm,
            dt,
        )
        self.current_right_rpm = self._ramp_rpm(
            self.current_right_rpm,
            right_rpm,
            dt,
        )

        left_rpm = self.current_left_rpm
        right_rpm = self.current_right_rpm

        if left_rpm == 0 and right_rpm == 0:
            if not self.is_stopped:
                self.send_stop_command(use_brake=False)
            return

        motor1_rpm, motor2_rpm = self._apply_motor_mapping(left_rpm, right_rpm)
        self._cancel_pending_brake()
        if self.connected:
            self.driver.send_rpm_command(
                motor1_rpm,
                motor2_rpm,
                return_type=2 if self.feedback_enabled else 0,
                clear_response=not self.feedback_enabled,
            )
            if self.feedback_enabled:
                self._read_and_update_motor_feedback()
            self.is_stopped = False
        elif self.dry_run:
            self.log_dry_run_command(motor1_rpm, motor2_rpm)
            self.is_stopped = False

    def watchdog_check_callback(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        if elapsed <= self.cmd_timeout:
            return False

        self.target_left_rpm = 0
        self.target_right_rpm = 0

        if not self.is_stopped:
            self.get_logger().warn(
                'cmd_vel timeout. Sending brake stop command.',
                throttle_duration_sec=1.0,
            )

        return True

    def _apply_motor_mapping(self, left_rpm, right_rpm):
        if self.cfg.SWAP_MOTORS:
            left_rpm, right_rpm = right_rpm, left_rpm

        left_rpm *= self.cfg.LEFT_RPM_SCALE
        right_rpm *= self.cfg.RIGHT_RPM_SCALE

        motor1_rpm = int(round(left_rpm * self.cfg.LEFT_FORWARD_SIGN))
        motor2_rpm = int(round(right_rpm * self.cfg.RIGHT_FORWARD_SIGN))
        return motor1_rpm, motor2_rpm

    def _feedback_to_wheel_rpm(self, motor1_rpm, motor2_rpm):
        left_scale = self.cfg.LEFT_RPM_SCALE or 1.0
        right_scale = self.cfg.RIGHT_RPM_SCALE or 1.0

        mapped_left = (
            float(motor1_rpm) / self.cfg.LEFT_FORWARD_SIGN / left_scale
        )
        mapped_right = (
            float(motor2_rpm) / self.cfg.RIGHT_FORWARD_SIGN / right_scale
        )

        if self.cfg.SWAP_MOTORS:
            mapped_left, mapped_right = mapped_right, mapped_left

        return mapped_left, mapped_right

    def _read_and_update_motor_feedback(self):
        feedback = self.driver.read_pnt_main_data_response(
            timeout=self.feedback_read_timeout
        )
        if feedback is None:
            self.get_logger().warn(
                'PID210 feedback read failed',
                throttle_duration_sec=1.0,
            )
            return False

        left_rpm, right_rpm = self._feedback_to_wheel_rpm(
            feedback['motor1_rpm'],
            feedback['motor2_rpm'],
        )
        self.actual_left_rpm = left_rpm
        self.actual_right_rpm = right_rpm
        self.last_feedback_time = self.get_clock().now()
        self._log_motor_feedback(feedback, left_rpm, right_rpm)
        return True

    def _log_motor_feedback(self, feedback, left_rpm, right_rpm):
        now = self.get_clock().now()
        elapsed = (now - self.last_feedback_log_time).nanoseconds / 1e9
        if elapsed < self.feedback_log_period:
            return

        self.get_logger().info(
            'PID210 feedback: '
            f'motor1={feedback["motor1_rpm"]}, '
            f'motor2={feedback["motor2_rpm"]}, '
            f'target_wheel=({self.target_left_rpm:.0f},{self.target_right_rpm:.0f}), '
            f'ramped_wheel=({self.current_left_rpm:.0f},{self.current_right_rpm:.0f}), '
            f'status=({feedback["motor1_status"]},{feedback["motor2_status"]}), '
            f'left={left_rpm:.1f}, '
            f'right={right_rpm:.1f}'
        )
        self.last_feedback_log_time = now

    def _ramp_rpm(self, current_rpm, target_rpm, dt):
        current_rpm = float(current_rpm)
        target_rpm = float(target_rpm)
        delta = target_rpm - current_rpm
        if abs(delta) < 0.5:
            return int(round(target_rpm))

        rate = self._select_ramp_rate(current_rpm, target_rpm)
        max_step = max(rate * dt, 1.0)
        if abs(delta) <= max_step:
            return int(round(target_rpm))

        if delta > 0.0:
            return int(round(current_rpm + max_step))
        return int(round(current_rpm - max_step))

    def _select_ramp_rate(self, current_rpm, target_rpm):
        current_abs = abs(current_rpm)
        target_abs = abs(target_rpm)

        if target_abs > current_abs:
            return max(self.cfg.ACCEL_RPM_PER_SEC, 1.0)
        return max(self.cfg.DECEL_RPM_PER_SEC, 1.0)

    def _send_brake_stop(self):
        self.driver.send_rpm_command(0, 0)
        self.driver.send_param(PID_COMMAND, CMD_BRAKE, 1)
        self.driver.send_rpm_command(0, 0)

    def _schedule_brake_command(self):
        self._cancel_pending_brake()

        delay_sec = max(self.cfg.BRAKE_DELAY_SEC, 0.0)
        if delay_sec <= 0.0:
            self._send_brake_stop()
            return

        self.pending_brake_timer = self.create_timer(
            delay_sec,
            self._delayed_brake_callback,
        )
        self.get_logger().info(
            f'brake command scheduled after {delay_sec:.2f}s'
        )

    def _delayed_brake_callback(self):
        if self.pending_brake_timer is not None:
            self.pending_brake_timer.cancel()
            self.pending_brake_timer = None

        if self.connected:
            self.driver.send_param(PID_COMMAND, CMD_BRAKE, 1)
            self.driver.send_rpm_command(0, 0)
            self.get_logger().info('delayed brake command sent')

    def _cancel_pending_brake(self):
        if self.pending_brake_timer is None:
            return

        self.pending_brake_timer.cancel()
        self.pending_brake_timer = None

    def send_stop_command(self, use_brake=None):
        if use_brake is None:
            use_brake = self.cfg.USE_CMD_BRAKE_ON_STOP

        if self.connected:
            if use_brake:
                self.driver.send_rpm_command(0, 0)
                self._schedule_brake_command()
            else:
                self._cancel_pending_brake()
                self.driver.send_rpm_command(0, 0)
        elif self.dry_run:
            self.log_dry_run_command(0, 0)

        self.is_stopped = True
        self.current_left_rpm = 0
        self.current_right_rpm = 0
        self.actual_left_rpm = 0
        self.actual_right_rpm = 0

    def apply_safety_gate(self, left_rpm, right_rpm):
        if self.directional_enabled:
            return self.apply_directional_gate(left_rpm, right_rpm)
        if not self.safety_stop_enabled:
            self.safety_stop_active = False
            return left_rpm, right_rpm

        obstacle_fresh, decision, distance, topic = self.get_obstacle_state()
        should_stop = False

        if not obstacle_fresh:
            should_stop = self.stop_on_unknown
        elif decision == 'obstacle':
            should_stop = True
        elif decision == 'unknown':
            should_stop = self.stop_on_unknown

        if should_stop and not self.safety_stop_active:
            self.get_logger().warn(
                'safety stop active: '
                f'topic={topic}, '
                f'decision={decision}, '
                f'distance={distance:.3f}'
            )
        elif not should_stop and self.safety_stop_active:
            self.get_logger().info('safety stop released')

        self.safety_stop_active = should_stop
        if should_stop:
            return 0, 0
        return left_rpm, right_rpm

    def apply_directional_gate(self, left_rpm, right_rpm):
        if left_rpm == 0 and right_rpm == 0:
            # Zero output is always permitted; only an explicit command clears latch.
            self.safety_stop_active = True
            return 0, 0
        allowed, reason = False, 'scan_or_tf_stale'
        if self.directional_scan is not None and self.directional_stamp is not None:
            age = (self.get_clock().now() - self.directional_stamp).nanoseconds / 1e9
            if 0 <= age <= 0.3:
                key = (left_rpm, right_rpm)
                if key not in self.directional_cache:
                    self.directional_cache[key] = self.directional_scan.check(
                        *self.kinematics.forward_kinematics(*key))
                allowed, reason = self.directional_cache[key]
                # Ramping must not carry the robot along an unsafe old direction.
                current = (self.current_left_rpm, self.current_right_rpm)
                if allowed and current != key and any(current):
                    if current not in self.directional_cache:
                        self.directional_cache[current] = self.directional_scan.check(
                            *self.kinematics.forward_kinematics(*current))
                    allowed, reason = self.directional_cache[current]
                age = (self.get_clock().now() - self.directional_stamp).nanoseconds / 1e9
                if not 0 <= age <= 0.3:
                    allowed, reason = False, 'scan_or_tf_stale'
        allowed = self.directional_latch.evaluate(allowed)
        if not allowed and reason == 'clear':
            reason = 'release_key_or_press_k_then_command_again'
        self.safety_stop_active = not allowed
        if reason != self.directional_reason:
            self.get_logger().info(f'directional safety: {reason}')
            self.directional_reason = reason
        return (left_rpm, right_rpm) if allowed else (0, 0)

    def get_obstacle_state(self):
        now = self.get_clock().now()
        fresh_states = []
        for topic, state in self.obstacle_states.items():
            elapsed = (now - state['time']).nanoseconds / 1e9
            if elapsed <= self.obstacle_timeout:
                fresh_states.append((topic, state))

        if not fresh_states:
            return False, 'unknown', math.inf, 'none'

        for topic, state in fresh_states:
            if state['decision'] == 'obstacle':
                return True, 'obstacle', state['distance'], topic

        for topic, state in fresh_states:
            if state['decision'] == 'unknown':
                return True, 'unknown', state['distance'], topic

        topic, state = fresh_states[0]
        return True, 'clear', state['distance'], topic

    def is_obstacle_fresh(self):
        if self.last_obstacle_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self.last_obstacle_time
        ).nanoseconds / 1e9
        return elapsed <= self.obstacle_timeout

    def log_dry_run_command(self, left_rpm, right_rpm):
        now = self.get_clock().now()
        elapsed = (now - self.last_dry_run_log_time).nanoseconds / 1e9
        if elapsed < self.dry_run_log_period:
            return

        self.get_logger().info(
            f'dry_run rpm left={left_rpm}, right={right_rpm}, '
            f'safety_stop={self.safety_stop_active}'
        )
        self.last_dry_run_log_time = now

    def _has_fresh_feedback(self, now):
        if not self.feedback_enabled or self.last_feedback_time is None:
            return False

        elapsed = (now - self.last_feedback_time).nanoseconds / 1e9
        return elapsed <= max(self.feedback_timeout, 0.0)

    def _get_odom_rpm_source(self, now):
        if self._has_fresh_feedback(now):
            return self.actual_left_rpm, self.actual_right_rpm, 'feedback'

        left_cmd, right_cmd = self.apply_safety_gate(
            self.target_left_rpm,
            self.target_right_rpm,
        )
        return left_cmd, right_cmd, 'open_loop'

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        left_cmd, right_cmd, _source = self._get_odom_rpm_source(now)
        linear_v, angular_w = self.kinematics.forward_kinematics(
            left_cmd,
            right_cmd,
        )

        self.theta += angular_w * dt
        self.x += linear_v * math.cos(self.theta) * dt
        self.y += linear_v * math.sin(self.theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        self._cancel_pending_brake()
        self.send_stop_command(use_brake=False)
        if self.driver is not None:
            self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
