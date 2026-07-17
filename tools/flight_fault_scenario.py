#!/usr/bin/env python3
"""Drive one deterministic FlightManager fault scenario and assert safety behavior."""

from __future__ import annotations

import argparse
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import FlightStatus


def is_zero(message: Twist) -> bool:
    values = (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z,
    )
    return all(abs(value) < 1e-6 for value in values)


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__('flight_fault_scenario')
        self.state = ''
        self.status_events: list[tuple[float, str]] = []
        self.commands: list[tuple[float, bool]] = []
        self.create_subscription(FlightStatus, '/flight/status', self._status, 10)
        self.create_subscription(Twist, '/tello/cmd_vel', self._command, 10)

    def _status(self, message: FlightStatus) -> None:
        self.state = message.state_name
        self.status_events.append((time.monotonic(), message.state_name))

    def _command(self, message: Twist) -> None:
        self.commands.append((time.monotonic(), is_zero(message)))

    def wait_for(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'timed out waiting for {description}; state={self.state}')

    def call(self, service_name: str, service_type, request):
        client = self.create_client(service_type, service_name)
        self.wait_for(
            lambda: client.wait_for_service(timeout_sec=0.0), 5.0, f'{service_name} service'
        )
        future = client.call_async(request)
        self.wait_for(lambda: future.done(), 5.0, f'{service_name} response')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'{service_name} failed: {getattr(response, "message", "no response")}')
        return response

    def collect_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def assert_zero_sequence(self, since: float, *, before: float | None = None) -> None:
        samples = [
            zero for timestamp, zero in self.commands
            if timestamp >= since and (before is None or timestamp <= before)
        ]
        if len(samples) < 3 or not all(samples[-3:]):
            raise AssertionError(f'expected at least three consecutive zero commands, got {samples}')


def set_bool(value: bool) -> SetBool.Request:
    request = SetBool.Request()
    request.data = value
    return request


def run_scenario(driver: ScenarioDriver, scenario: str) -> None:
    driver.wait_for(lambda: driver.state == 'READY', 8.0, 'READY')
    driver.call('/flight/takeoff', Trigger, Trigger.Request())
    driver.wait_for(lambda: driver.state == 'TRACKING', 5.0, 'TRACKING')
    driver.wait_for(
        lambda: any(not zero for _, zero in driver.commands), 3.0, 'non-zero approved command'
    )

    if scenario == 'tracking':
        fault_at = time.monotonic()
        driver.call('/mock/pause_tracking', SetBool, set_bool(True))
        driver.wait_for(lambda: driver.state == 'LOST_TARGET', 3.0, 'LOST_TARGET')
        driver.collect_for(0.35)
        driver.assert_zero_sequence(fault_at)
        driver.call('/mock/reset_faults', Trigger, Trigger.Request())
        driver.wait_for(lambda: driver.state == 'TRACKING', 2.0, 'tracking recovery')
        return

    service = {
        'video': '/mock/pause_video',
        'telemetry': '/mock/pause_telemetry',
    }.get(scenario)
    fault_at = time.monotonic()
    if service:
        driver.call(service, SetBool, set_bool(True))
    elif scenario == 'emergency':
        driver.call('/flight/emergency_stop', Trigger, Trigger.Request())
    else:
        raise ValueError(f'unknown scenario: {scenario}')
    driver.wait_for(lambda: driver.state == 'LANDED', 5.0, 'LANDED')
    landed_at = next(timestamp for timestamp, state in driver.status_events if timestamp >= fault_at and state == 'LANDED')
    driver.collect_for(0.25)
    driver.assert_zero_sequence(fault_at, before=None if scenario == 'emergency' else landed_at)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=['tracking', 'video', 'telemetry', 'emergency'])
    args = parser.parse_args()
    rclpy.init()
    driver = ScenarioDriver()
    try:
        run_scenario(driver, args.scenario)
        print(f'{args.scenario} fault scenario passed')
    finally:
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
