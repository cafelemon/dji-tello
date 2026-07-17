import glob
import platform
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from .system_metrics import (
    cpu_percent,
    read_cpu_times,
    read_gpu_percent,
    read_max_temperature,
    read_memory,
    read_process_rss,
    read_thermal_throttle,
)


def key_value(key, value):
    return KeyValue(key=key, value=str(value))


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('tello_system_monitor')
        period_s = float(self.declare_parameter('diagnostics_period_s', 1.0).value)
        if period_s <= 0.0:
            raise ValueError('diagnostics_period_s must be positive')
        self.warning_temperature_c = float(
            self.declare_parameter('warning_temperature_c', 80.0).value)
        self.critical_temperature_c = float(
            self.declare_parameter('critical_temperature_c', 90.0).value)
        self.critical_nodes = list(
            self.declare_parameter(
                'critical_nodes',
                ['tello_transport', 'tello_flight_manager', 'tello_tracker'],
            ).value
        )
        self.publisher = self.create_publisher(
            DiagnosticArray, '/system/diagnostics', 10)
        self.previous_cpu = read_cpu_times()
        self.last_power_query = 0.0
        self.power_mode = 'unavailable'
        self.timer = self.create_timer(period_s, self.publish_diagnostics)

    def query_power_mode(self):
        now = time.monotonic()
        if now - self.last_power_query < 30.0:
            return self.power_mode
        self.last_power_query = now
        executable = shutil.which('nvpmodel')
        if executable is None:
            self.power_mode = 'unavailable'
            return self.power_mode
        try:
            result = subprocess.run(
                [executable, '-q'], capture_output=True, text=True,
                check=False, timeout=1.0)
            output = (result.stdout or result.stderr).strip().replace('\n', '; ')
            self.power_mode = output if result.returncode == 0 and output else 'query_failed'
        except (OSError, subprocess.TimeoutExpired):
            self.power_mode = 'query_failed'
        return self.power_mode

    def publish_diagnostics(self):
        current_cpu = read_cpu_times()
        cpu_usage = cpu_percent(self.previous_cpu, current_cpu)
        self.previous_cpu = current_cpu
        memory = read_memory()
        gpu_usage = read_gpu_percent([
            Path('/sys/devices/gpu.0/load'),
            Path('/sys/devices/platform/17000000.gpu/load'),
            Path('/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/load'),
        ])
        temperature = read_max_temperature(
            Path(path) for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'))
        throttle = read_thermal_throttle(
            Path(path) for path in glob.glob('/sys/class/thermal/cooling_device*'))
        available_nodes = set(self.get_node_names())
        missing_nodes = sorted(set(self.critical_nodes) - available_nodes)

        level = DiagnosticStatus.OK
        message = 'system healthy'
        if missing_nodes or throttle == 'active':
            level = DiagnosticStatus.WARN
            message = 'degraded system state'
        if temperature is not None and temperature >= self.warning_temperature_c:
            level = max(level, DiagnosticStatus.WARN)
            message = 'temperature warning'
        if temperature is not None and temperature >= self.critical_temperature_c:
            level = DiagnosticStatus.ERROR
            message = 'temperature critical'

        status = DiagnosticStatus(
            level=level,
            name='tello_system/host',
            hardware_id=platform.node() or 'unknown-host',
            message=message,
        )
        status.values = [
            key_value('cpu_percent', f'{cpu_usage:.2f}'),
            key_value('gpu_percent', 'unavailable' if gpu_usage is None else f'{gpu_usage:.2f}'),
            key_value('ram_used_percent', f'{memory.used_percent:.2f}'),
            key_value('ram_total_bytes', memory.total_bytes),
            key_value('process_rss_bytes', read_process_rss()),
            key_value('temperature_c', 'unavailable' if temperature is None else f'{temperature:.2f}'),
            key_value('power_mode', self.query_power_mode()),
            key_value('thermal_throttle', throttle),
            key_value('process_health', 'healthy' if not missing_nodes else 'degraded'),
            key_value('missing_nodes', ','.join(missing_nodes) if missing_nodes else 'none'),
        ]
        message_array = DiagnosticArray()
        message_array.header.stamp = self.get_clock().now().to_msg()
        message_array.status = [status]
        self.publisher.publish(message_array)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
