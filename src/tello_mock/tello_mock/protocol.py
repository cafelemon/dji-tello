"""Stateful subset of Tello SDK behavior used by the UDP mock."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MockTelloProtocol:
    command_mode: bool = False
    streaming: bool = False
    airborne: bool = False
    takeoff_count: int = 0
    land_count: int = 0

    def handle(self, command: str) -> str | None:
        normalized = command.strip().lower()
        if normalized.startswith('rc '):
            return None
        if normalized == 'command':
            self.command_mode = True
            return 'ok'
        if not self.command_mode:
            return 'error command mode required'
        if normalized == 'streamon':
            self.streaming = True
            return 'ok'
        if normalized == 'streamoff':
            self.streaming = False
            return 'ok'
        if normalized == 'takeoff':
            if self.airborne:
                return 'error already airborne'
            self.airborne = True
            self.takeoff_count += 1
            return 'ok'
        if normalized == 'land':
            if not self.airborne:
                return 'error not airborne'
            self.airborne = False
            self.land_count += 1
            return 'ok'
        if normalized == 'emergency':
            self.airborne = False
            return 'ok'
        return 'error unsupported command'
