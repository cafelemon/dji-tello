"""Pure safety state machine, independent of ROS for deterministic tests."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class FlightState(IntEnum):
    DISCONNECTED = 0
    CONNECTED = 1
    READY = 2
    TAKING_OFF = 3
    TRACKING = 4
    LOST_TARGET = 5
    ERROR = 6
    LANDING = 7
    LANDED = 8


@dataclass(frozen=True)
class TransitionResult:
    accepted: bool
    reason: str


class FlightStateMachine:
    """State transitions and timeout actions for safe command arbitration."""

    def __init__(self, reacquire_timeout_s: float = 2.0, lost_land_timeout_s: float = 5.0) -> None:
        if reacquire_timeout_s < 0.0 or lost_land_timeout_s < reacquire_timeout_s:
            raise ValueError('lost-target timeout configuration is invalid')
        self.state = FlightState.DISCONNECTED
        self.reason = 'startup'
        self.airborne = False
        self.reacquire_timeout_s = reacquire_timeout_s
        self.lost_land_timeout_s = lost_land_timeout_s
        self._lost_since: float | None = None
        self._landing_requested = False

    def _transition(self, state: FlightState, reason: str) -> None:
        self.state = state
        self.reason = reason

    def update_links(self, control: bool, telemetry: bool, video: bool) -> None:
        healthy = control and telemetry and video
        if healthy:
            if self.state == FlightState.DISCONNECTED:
                self._transition(FlightState.CONNECTED, 'transport connected')
            elif self.state == FlightState.CONNECTED:
                self._transition(FlightState.READY, 'required links healthy')
            return

        if self.state in {
            FlightState.TAKING_OFF,
            FlightState.TRACKING,
            FlightState.LOST_TARGET,
            FlightState.LANDING,
        } or self.airborne:
            self._transition(FlightState.ERROR, 'required link unhealthy')
        elif self.state not in {FlightState.LANDED, FlightState.ERROR}:
            self._transition(FlightState.DISCONNECTED, 'required link unhealthy')

    def request_takeoff(self, battery_percent: float, minimum_battery_percent: float) -> TransitionResult:
        if self.state != FlightState.READY:
            return TransitionResult(False, f'takeoff requires READY, current={self.state.name}')
        if battery_percent < minimum_battery_percent:
            return TransitionResult(False, 'battery below takeoff threshold')
        self._transition(FlightState.TAKING_OFF, 'takeoff requested')
        return TransitionResult(True, self.reason)

    def takeoff_result(self, success: bool, reason: str) -> None:
        if self.state != FlightState.TAKING_OFF:
            return
        if success:
            self.airborne = True
            self._transition(FlightState.TRACKING, reason or 'takeoff complete')
        else:
            self._transition(FlightState.ERROR, reason or 'takeoff failed')

    def request_land(self, reason: str = 'land requested') -> TransitionResult:
        if self.state in {FlightState.LANDING, FlightState.LANDED}:
            return TransitionResult(False, f'already {self.state.name}')
        if not self.airborne and self.state not in {
            FlightState.TAKING_OFF,
            FlightState.TRACKING,
            FlightState.LOST_TARGET,
            FlightState.ERROR,
        }:
            return TransitionResult(False, 'aircraft is not airborne')
        self._landing_requested = True
        self._transition(FlightState.LANDING, reason)
        return TransitionResult(True, self.reason)

    def land_result(self, success: bool, reason: str) -> None:
        if success:
            self.airborne = False
            self._landing_requested = False
            self._transition(FlightState.LANDED, reason or 'landing complete')
        else:
            self._transition(FlightState.ERROR, reason or 'landing failed')

    def emergency(self, reason: str = 'emergency stop requested') -> None:
        self._landing_requested = True
        self._transition(FlightState.ERROR, reason)

    def update_tracking(self, visible: bool, now: float) -> None:
        if self.state == FlightState.TRACKING and not visible:
            self._lost_since = now
            self._transition(FlightState.LOST_TARGET, 'target lost; holding position')
        elif self.state == FlightState.LOST_TARGET and visible and self._lost_since is not None:
            if now - self._lost_since <= self.reacquire_timeout_s:
                self._lost_since = None
                self._transition(FlightState.TRACKING, 'target reacquired')

    def tick(self, now: float) -> str | None:
        if self.state == FlightState.LOST_TARGET and self._lost_since is not None:
            if now - self._lost_since >= self.lost_land_timeout_s and not self._landing_requested:
                self._landing_requested = True
                return 'land'
        if self.state == FlightState.ERROR and self.airborne and not self._landing_requested:
            self._landing_requested = True
            return 'land'
        return None

    @property
    def motion_allowed(self) -> bool:
        return self.state == FlightState.TRACKING and self.airborne
