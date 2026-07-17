#!/usr/bin/env python3
"""Assert observable outputs from the offline vision pipeline."""

from __future__ import annotations

import argparse
import json
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tello_interfaces.msg import TrackingStatus


class OfflineProbe(Node):
    def __init__(self) -> None:
        super().__init__('offline_probe')
        self.image_count = 0
        self.visible_count = 0
        self.nonzero_command_count = 0
        self.diagnostics: dict[str, str] = {}
        self.create_subscription(Image, '/yolo/image_out', self._image, qos_profile_sensor_data)
        self.create_subscription(TrackingStatus, '/tracking/status', self._status, 10)
        self.create_subscription(TwistStamped, '/tracking/cmd_vel', self._command, 10)
        self.create_subscription(DiagnosticArray, '/tracking/diagnostics', self._diagnostic, 10)

    def _image(self, _message: Image) -> None:
        self.image_count += 1

    def _status(self, message: TrackingStatus) -> None:
        if message.locked and message.visible:
            self.visible_count += 1

    def _command(self, message: TwistStamped) -> None:
        values = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.z,
        )
        if any(abs(value) > 1e-6 for value in values):
            self.nonzero_command_count += 1

    def _diagnostic(self, message: DiagnosticArray) -> None:
        if message.status:
            self.diagnostics = {item.key: item.value for item in message.status[0].values}

    @property
    def complete(self) -> bool:
        return (
            self.image_count > 0
            and self.visible_count > 0
            and self.nonzero_command_count > 0
            and int(self.diagnostics.get('dropped_frames', '0')) > 0
            and int(self.diagnostics.get('error_count', '1')) == 0
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    probe = OfflineProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not probe.complete:
            rclpy.spin_once(probe, timeout_sec=0.1)
        result = {
            'image_count': probe.image_count,
            'visible_count': probe.visible_count,
            'nonzero_command_count': probe.nonzero_command_count,
            'diagnostics': probe.diagnostics,
            'passed': probe.complete,
        }
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        if not probe.complete:
            raise RuntimeError(f'offline pipeline did not satisfy acceptance: {result}')
        print('offline vision smoke passed')
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
