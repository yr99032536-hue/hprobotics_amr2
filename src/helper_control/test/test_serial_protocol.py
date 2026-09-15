from unittest.mock import Mock, patch

from helper_control.md200t_driver import MD200TDriver


def test_start_stop_never_write_acceleration_register():
    driver = MD200TDriver()
    driver.serial_port = Mock(is_open=True)
    driver.send_param = Mock(return_value=True)
    driver.send_rpm_command = Mock(return_value=True)
    with patch('helper_control.md200t_driver.time.sleep'):
        assert driver.initialize_motor()
        driver.stop_motor()
    assert all(call.args[0] != 124 for call in driver.send_param.call_args_list)
    driver.send_rpm_command.assert_called_with(0, 0)


def test_register_read_uses_pid4_and_skips_other_response():
    driver = MD200TDriver()
    def frame(pid, payload):
        raw = bytes([184, 183, 1, pid, len(payload)]) + payload
        return raw + bytes([(-sum(raw)) & 255])
    response = bytearray(frame(154, b'\x01\x00') + frame(143, b'\x78\x00'))
    port = Mock(is_open=True, timeout=0.1)
    port.read.side_effect = lambda n: bytes([response.pop(0)]) if response else b''
    driver.serial_port = port
    assert driver.read_pid_data(143, 2) == b'\x78\x00'
    packet = bytes([183, 184, 1, 4, 1, 143])
    port.write.assert_called_once_with(packet + bytes([(-sum(packet)) & 255]))
    assert port.timeout == 0.1


def test_parser_rejects_wrong_total_length():
    driver = MD200TDriver()
    raw = bytes([184, 183, 1, 143, 2, 120])
    packet = raw + bytes([(-sum(raw)) & 255])
    assert driver._parse_pid_response(packet, 143, 2) is None
