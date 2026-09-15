import struct
import threading
import time

try:
    import serial
except ImportError:
    serial = None


class MD200TDriver:
    """Low-level MD200T RS485 serial driver."""

    def __init__(
        self,
        port='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0',
        baudrate=57600,
        robot_id=1,
        max_rpm=4000,
        com_watch_delay=10,
    ):
        self.port = port
        self.baudrate = baudrate
        self.robot_id = robot_id
        self.max_rpm = max_rpm
        self.com_watch_delay = com_watch_delay
        self.serial_port = None
        self.last_error = ''
        self.lock = threading.Lock()

        self.RMID = 183
        self.TMID = 184
        self.PID_PNT_VEL_CMD = 207
        self.PID_TQ_OFF = 5
        # PID 124 is MIN_SSSD (a TWO-byte acceleration/deceleration limit),
        # not a broadcast switch. Never write it during startup/shutdown.
        self.PID_REQ_PID_DATA = 4
        self.PID_COM_WATCH_DELAY = 185
        self.PID_COMMAND = 10
        self.PID_MAIN_DATA = 193
        self.PID_MAIN_DATA2 = 200
        self.PID_PNT_MAIN_DATA = 210
        self.CMD_BRAKE = 4

    def connect(self):
        if serial is None:
            self.last_error = 'pyserial is not installed. Install python3-serial.'
            print(f'[ERROR] {self.last_error}', flush=True)
            return False

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            self.last_error = ''
            return self.serial_port.is_open
        except Exception as exc:
            self.last_error = str(exc)
            print(
                f'[ERROR] MD200T connection failed on {self.port}: {exc}',
                flush=True,
            )
            return False

    def _calculate_checksum(self, packet_bytes):
        total_sum = sum(packet_bytes) & 0xFF
        return (~total_sum + 1) & 0xFF

    def send_param(self, pid, data, byte_size=1):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        if byte_size == 1:
            data_payload = bytes([data])
        elif byte_size == 2:
            data_payload = struct.pack('<h', data)
        else:
            raise ValueError('byte_size must be 1 or 2')

        packet_no_chk = (
            bytes([self.RMID, self.TMID, self.robot_id, pid, byte_size])
            + data_payload
        )
        checksum = self._calculate_checksum(packet_no_chk)

        with self.lock:
            try:
                self.serial_port.write(packet_no_chk + bytes([checksum]))
                time.sleep(0.05)
                return True
            except Exception as exc:
                print(f'[ERROR] parameter send failed pid={pid}: {exc}')
                return False

    def initialize_motor(self):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        self.send_param(81, 0, 1)
        self.send_param(92, 0, 1)
        self.send_param(self.PID_COM_WATCH_DELAY, self.com_watch_delay, 2)
        return True

    def brake_motor(self):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        self.send_rpm_command(0, 0)
        time.sleep(0.05)
        return self.send_param(self.PID_COMMAND, self.CMD_BRAKE, 1)

    def send_rpm_command(
        self,
        left_rpm,
        right_rpm,
        return_type=0,
        clear_response=True,
    ):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        left_rpm_clamped = max(
            min(int(left_rpm), self.max_rpm),
            -self.max_rpm,
        )
        right_rpm_clamped = max(
            min(int(right_rpm), self.max_rpm),
            -self.max_rpm,
        )

        motor1_bytes = struct.pack('<h', left_rpm_clamped)
        motor2_bytes = struct.pack('<h', right_rpm_clamped)

        data_payload = (
            bytes([1])
            + motor1_bytes
            + bytes([1])
            + motor2_bytes
            + bytes([return_type & 0xFF])
        )
        packet_no_chk = (
            bytes([
                self.RMID,
                self.TMID,
                self.robot_id,
                self.PID_PNT_VEL_CMD,
                7,
            ])
            + data_payload
        )
        checksum = self._calculate_checksum(packet_no_chk)

        with self.lock:
            try:
                self.serial_port.write(packet_no_chk + bytes([checksum]))
                if clear_response:
                    self.serial_port.read(self.serial_port.in_waiting)
                return True
            except Exception as exc:
                print(f'[ERROR] RPM command send failed: {exc}')
                return False

    def read_raw_available(self, timeout=0.2):
        if not self.serial_port or not self.serial_port.is_open:
            return b''

        deadline = time.monotonic() + max(float(timeout), 0.01)
        data = bytearray()

        with self.lock:
            original_timeout = self.serial_port.timeout
            try:
                while time.monotonic() < deadline:
                    # The port defaults to 100 ms, longer than the motor loop's
                    # 20 ms feedback budget. Bound each blocking read by the
                    # remaining budget so scan/TF callbacks can run on time.
                    remaining = max(0.0, deadline - time.monotonic())
                    self.serial_port.timeout = (
                        remaining if original_timeout is None
                        else min(original_timeout, remaining)
                    )
                    waiting = self.serial_port.in_waiting
                    chunk = self.serial_port.read(waiting or 1)
                    if chunk:
                        data.extend(chunk)
            except Exception as exc:
                print(f'[ERROR] raw read failed: {exc}')
                return b''
            finally:
                self.serial_port.timeout = original_timeout

        return bytes(data)

    def read_pnt_main_data_response(self, timeout=0.2):
        raw = self.read_raw_available(timeout=timeout)
        return self.parse_pnt_main_data_from_raw(raw)

    def parse_pnt_main_data_from_raw(self, raw):
        packet_size = 24
        for start in range(0, max(len(raw) - packet_size + 1, 0)):
            packet = raw[start:start + packet_size]
            data = self._parse_pid_response(packet, self.PID_PNT_MAIN_DATA, 18)
            if data is None:
                continue

            return {
                'motor1_rpm': struct.unpack('<h', data[0:2])[0],
                'motor1_current_raw': struct.unpack('<h', data[2:4])[0],
                'motor1_status': data[4],
                'motor1_position': struct.unpack('<i', data[5:9])[0],
                'motor2_rpm': struct.unpack('<h', data[9:11])[0],
                'motor2_current_raw': struct.unpack('<h', data[11:13])[0],
                'motor2_status': data[13],
                'motor2_position': struct.unpack('<i', data[14:18])[0],
                'raw': raw,
            }

        return None

    def read_pid_data(self, pid, expected_size, timeout=0.2):
        if not self.serial_port or not self.serial_port.is_open:
            return None

        request_no_chk = bytes([
            self.RMID,
            self.TMID,
            self.robot_id,
            self.PID_REQ_PID_DATA,
            1,
            pid,
        ])
        request = request_no_chk + bytes([
            self._calculate_checksum(request_no_chk)
        ])

        deadline = time.monotonic() + max(float(timeout), 0.01)
        response = bytearray()
        expected_total = 6 + expected_size

        with self.lock:
            original_timeout = self.serial_port.timeout
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.write(request)

                while time.monotonic() < deadline:
                    remaining = max(0.0, deadline - time.monotonic())
                    self.serial_port.timeout = (
                        remaining if original_timeout is None
                        else min(original_timeout, remaining)
                    )
                    chunk = self.serial_port.read(1)
                    if not chunk:
                        continue

                    response.extend(chunk)
                    while response and response[0] != self.TMID:
                        response.pop(0)

                    if len(response) >= 2 and response[1] != self.RMID:
                        response.pop(0)
                        continue

                    if len(response) >= 5:
                        data_size = response[4]
                        expected_total = 6 + data_size
                        if len(response) >= expected_total:
                            packet = bytes(response[:expected_total])
                            parsed = self._parse_pid_response(
                                packet,
                                pid,
                                expected_size,
                            )
                            if parsed is not None:
                                return parsed
                            del response[:expected_total]
            except Exception as exc:
                print(f'[ERROR] PID read failed pid={pid}: {exc}')
                return None
            finally:
                self.serial_port.timeout = original_timeout

        return None

    def _parse_pid_response(self, packet, pid, expected_size):
        if len(packet) < 6:
            return None

        packet_no_chk = packet[:-1]
        checksum = packet[-1]
        if self._calculate_checksum(packet_no_chk) != checksum:
            return None

        if packet[0] != self.TMID or packet[1] != self.RMID:
            return None
        if packet[2] != self.robot_id or packet[3] != pid:
            return None
        if packet[4] != expected_size:
            return None
        if len(packet) != expected_size + 6:
            return None

        return packet[5:-1]

    def read_main_data(self, pid, timeout=0.2):
        data = self.read_pid_data(pid, 17, timeout=timeout)
        if data is None:
            return None

        return {
            'rpm': struct.unpack('<h', data[0:2])[0],
            'current_raw': struct.unpack('<h', data[2:4])[0],
            'control_type': data[4],
            'ref_rpm': struct.unpack('<h', data[5:7])[0],
            'control_output': struct.unpack('<h', data[7:9])[0],
            'controller_status': data[9],
            'position': struct.unpack('<i', data[10:14])[0],
            'brake_output': data[14],
            'temperature_c': data[15],
            'status2': data[16],
        }

    def read_motor_feedback(self, timeout=0.2):
        motor1 = self.read_main_data(self.PID_MAIN_DATA, timeout=timeout)
        motor2 = self.read_main_data(self.PID_MAIN_DATA2, timeout=timeout)
        if motor1 is None or motor2 is None:
            return None

        return {
            'motor1': motor1,
            'motor2': motor2,
        }

    def stop_motor(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        self.send_rpm_command(0, 0)
        time.sleep(0.1)

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.stop_motor()
            time.sleep(0.1)
            self.serial_port.close()
