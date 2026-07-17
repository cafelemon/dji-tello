from pathlib import Path

from tello_system_monitor.system_metrics import (
    cpu_percent,
    parse_cpu_times,
    parse_meminfo,
    read_gpu_percent,
    read_max_temperature,
    read_thermal_throttle,
)


def test_cpu_usage_uses_deltas():
    previous = parse_cpu_times('cpu  10 0 5 85 0 0 0 0')
    current = parse_cpu_times('cpu  20 0 10 170 0 0 0 0')
    assert cpu_percent(previous, current) == 15.0


def test_memory_usage_uses_available_memory():
    memory = parse_meminfo('MemTotal: 1000 kB\nMemAvailable: 250 kB\n')
    assert memory.total_bytes == 1024000
    assert memory.used_percent == 75.0


def test_jetson_metrics_parse_sysfs(tmp_path: Path):
    gpu = tmp_path / 'load'
    gpu.write_text('375\n', encoding='utf-8')
    cold = tmp_path / 'cold'
    hot = tmp_path / 'hot'
    cold.write_text('43000\n', encoding='utf-8')
    hot.write_text('72.5\n', encoding='utf-8')
    assert read_gpu_percent([gpu]) == 37.5
    assert read_max_temperature([cold, hot]) == 72.5


def test_throttle_state_only_uses_frequency_cooling_devices(tmp_path: Path):
    device = tmp_path / 'cooling_device0'
    device.mkdir()
    (device / 'type').write_text('thermal-cpufreq\n', encoding='utf-8')
    (device / 'cur_state').write_text('1\n', encoding='utf-8')
    assert read_thermal_throttle([device]) == 'active'
    (device / 'cur_state').write_text('0\n', encoding='utf-8')
    assert read_thermal_throttle([device]) == 'inactive'
    assert read_thermal_throttle([]) == 'unknown'
