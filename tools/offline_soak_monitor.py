#!/usr/bin/env python3
"""Collect and gate long-running offline vision stability evidence."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import time

from diagnostic_msgs.msg import DiagnosticArray
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tello_interfaces.msg import TrackingStatus


class SoakMonitor(Node):
    def __init__(self) -> None:
        super().__init__('offline_soak_monitor')
        self.images = 0
        self.visible = 0
        self.transitions = 0
        self.last_visibility: bool | None = None
        self.diagnostics: dict[str, str] = {}
        self.create_subscription(Image, '/yolo/image_out', self._image, qos_profile_sensor_data)
        self.create_subscription(TrackingStatus, '/tracking/status', self._status, 10)
        self.create_subscription(DiagnosticArray, '/tracking/diagnostics', self._diagnostic, 10)

    def _image(self, _message: Image) -> None:
        self.images += 1

    def _status(self, message: TrackingStatus) -> None:
        visible = bool(message.locked and message.visible)
        self.visible += int(visible)
        if self.last_visibility is not None and visible != self.last_visibility:
            self.transitions += 1
        self.last_visibility = visible

    def _diagnostic(self, message: DiagnosticArray) -> None:
        if message.status:
            self.diagnostics = {item.key: item.value for item in message.status[0].values}


def process_tree_rss_kb(root_pid: int) -> int:
    root = psutil.Process(root_pid)
    processes = [root, *root.children(recursive=True)]
    total = 0
    for process in processes:
        try:
            total += process.memory_info().rss
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return total // 1024


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--launch-pid', type=int, required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--sample-period', type=float, default=5.0)
    args = parser.parse_args()
    if args.duration < 1:
        raise ValueError('duration must be positive')
    rclpy.init()
    monitor = SoakMonitor()
    started = time.monotonic()
    next_sample = started
    samples: list[dict[str, float | int]] = []
    try:
        while time.monotonic() - started < args.duration:
            if not psutil.pid_exists(args.launch_pid):
                raise RuntimeError('offline launch exited before soak duration')
            rclpy.spin_once(monitor, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_sample:
                samples.append({
                    'elapsed_s': round(now - started, 3),
                    'rss_kb': process_tree_rss_kb(args.launch_pid),
                    'fps': float(monitor.diagnostics.get('fps', '0')),
                    'inference_ms': float(monitor.diagnostics.get('inference_ms', '0')),
                    'frame_age_ms': float(monitor.diagnostics.get('frame_age_ms', '0')),
                    'dropped_frames': int(monitor.diagnostics.get('dropped_frames', '0')),
                    'error_count': int(monitor.diagnostics.get('error_count', '0')),
                })
                next_sample = now + args.sample_period

        warmup_s = min(600.0, args.duration * 0.2)
        window_s = min(600.0, max(args.sample_period * 3.0, args.duration * 0.2))
        baseline = [s['rss_kb'] for s in samples if warmup_s <= s['elapsed_s'] < warmup_s + window_s]
        ending = [s['rss_kb'] for s in samples if s['elapsed_s'] >= args.duration - window_s]
        if not baseline or not ending:
            raise RuntimeError('insufficient RSS samples for stability gate')
        baseline_median = statistics.median(baseline)
        ending_median = statistics.median(ending)
        ending_steps = [right - left for left, right in zip(ending, ending[1:])]
        positive_ratio = (
            sum(step > 0 for step in ending_steps) / len(ending_steps) if ending_steps else 0.0
        )
        errors = max((int(sample['error_count']) for sample in samples), default=0)
        passed = (
            monitor.images > 0
            and monitor.visible > 0
            and errors == 0
            and ending_median <= baseline_median * 1.15
            and positive_ratio < 0.9
        )
        result = {
            'duration_s': args.duration,
            'images': monitor.images,
            'visible_status_count': monitor.visible,
            'visibility_transitions': monitor.transitions,
            'error_count': errors,
            'baseline_rss_median_kb': baseline_median,
            'ending_rss_median_kb': ending_median,
            'ending_positive_step_ratio': positive_ratio,
            'passed': passed,
        }
        with open(f'{args.output_dir}/metrics.csv', 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=samples[0].keys())
            writer.writeheader()
            writer.writerows(samples)
        with open(f'{args.output_dir}/summary.json', 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        if not passed:
            raise RuntimeError(f'offline soak acceptance failed: {result}')
        print(json.dumps(result, ensure_ascii=False))
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
