import math

import pytest

from tello_flight_manager.pid import Limits, PIDController


def test_pid_uses_dt_and_output_limit():
    pid = PIDController(1.0, 1.0, 0.0, output_limits=Limits(-2.0, 2.0))
    first = pid.update(-1.0, now=10.0)
    second = pid.update(-1.0, now=11.0)
    assert 1.0 <= first <= 2.0
    assert second == pytest.approx(2.0)


def test_pid_deadband_filter_and_reset():
    pid = PIDController(
        1.0, 0.0, 0.0, deadband=0.5, filter_alpha=0.5,
        output_limits=Limits(-10.0, 10.0),
    )
    assert pid.update(0.2, now=1.0) == 0.0
    assert pid.update(-2.0, now=2.0) == pytest.approx(1.0)
    pid.reset()
    assert pid.update(0.0, now=3.0) == 0.0


def test_pid_rejects_invalid_configuration():
    with pytest.raises(ValueError):
        PIDController(1.0, 0.0, 0.0, filter_alpha=0.0)
