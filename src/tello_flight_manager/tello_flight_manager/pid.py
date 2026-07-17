"""Time-aware PID controller used by flight control loops."""

from __future__ import annotations

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class Limits:
    minimum: float
    maximum: float

    def clamp(self, value: float) -> float:
        return max(self.minimum, min(self.maximum, value))


class PIDController:
    """PID with monotonic dt, anti-windup, deadband and output filtering."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        *,
        setpoint: float = 0.0,
        integral_limits: Limits = Limits(-100.0, 100.0),
        output_limits: Limits = Limits(-1.0, 1.0),
        deadband: float = 0.0,
        filter_alpha: float = 1.0,
        minimum_dt: float = 1e-3,
        maximum_dt: float = 1.0,
    ) -> None:
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError('filter_alpha must be in (0, 1]')
        if minimum_dt <= 0.0 or maximum_dt < minimum_dt:
            raise ValueError('invalid dt bounds')
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limits = integral_limits
        self.output_limits = output_limits
        self.deadband = abs(deadband)
        self.filter_alpha = filter_alpha
        self.minimum_dt = minimum_dt
        self.maximum_dt = maximum_dt
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: float | None = None
        self._previous_time: float | None = None
        self._filtered_output = 0.0

    def update(self, measurement: float, *, now: float | None = None) -> float:
        timestamp = time.monotonic() if now is None else now
        error = self.setpoint - measurement
        if abs(error) <= self.deadband:
            error = 0.0

        if self._previous_time is None:
            dt = self.minimum_dt
            derivative = 0.0
        else:
            dt = max(self.minimum_dt, min(self.maximum_dt, timestamp - self._previous_time))
            derivative = (error - (self._previous_error or 0.0)) / dt

        self._integral = self.integral_limits.clamp(self._integral + error * dt)
        raw_output = self.kp * error + self.ki * self._integral + self.kd * derivative
        limited = self.output_limits.clamp(raw_output)
        self._filtered_output = (
            self.filter_alpha * limited + (1.0 - self.filter_alpha) * self._filtered_output
        )
        self._previous_error = error
        self._previous_time = timestamp
        return self._filtered_output
