"""Serial feedback must not starve ROS safety and odometry callbacks."""

from unittest.mock import patch

import pytest

from helper_control.md200t_driver import MD200TDriver


class SimulatedPort:
    is_open = True

    def __init__(self, clock, data=b'', fail=False, timeout=0.1):
        self.clock = clock
        self.data = data
        self.fail = fail
        self.timeout = timeout

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        if self.fail:
            raise OSError('disconnected')
        if self.data:
            output, self.data = self.data[:count], self.data[count:]
            return output
        self.clock[0] += self.timeout if self.timeout is not None else 10.0
        return b''


@pytest.mark.parametrize('original_timeout', [0.1, None])
@pytest.mark.parametrize('data', [b'', b'partial', bytes(range(24))])
def test_feedback_respects_budget_and_preserves_received_data(original_timeout, data):
    clock = [0.0]
    driver = MD200TDriver()
    driver.serial_port = SimulatedPort(clock, data, timeout=original_timeout)
    with patch('helper_control.md200t_driver.time.monotonic', lambda: clock[0]):
        assert driver.read_raw_available(timeout=0.02) == data
    assert clock[0] <= 0.020001
    assert driver.serial_port.timeout == original_timeout


def test_disconnection_restores_timeout():
    clock = [0.0]
    driver = MD200TDriver()
    driver.serial_port = SimulatedPort(clock, fail=True)
    with patch('helper_control.md200t_driver.time.monotonic', lambda: clock[0]):
        assert driver.read_raw_available(timeout=0.02) == b''
    assert driver.serial_port.timeout == 0.1
