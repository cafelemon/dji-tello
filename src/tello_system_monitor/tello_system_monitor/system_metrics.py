from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class MemoryMetrics:
    total_bytes: int
    available_bytes: int

    @property
    def used_percent(self) -> float:
        if self.total_bytes <= 0:
            return 0.0
        return 100.0 * (self.total_bytes - self.available_bytes) / self.total_bytes


def parse_cpu_times(line: str) -> Tuple[int, int]:
    fields = line.split()
    if not fields or fields[0] != 'cpu' or len(fields) < 5:
        raise ValueError('invalid aggregate /proc/stat line')
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def cpu_percent(previous: Tuple[int, int], current: Tuple[int, int]) -> float:
    total_delta = current[0] - previous[0]
    idle_delta = current[1] - previous[1]
    if total_delta <= 0:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (total_delta - idle_delta) / total_delta))


def read_cpu_times(path: Path = Path('/proc/stat')) -> Tuple[int, int]:
    return parse_cpu_times(path.read_text(encoding='utf-8').splitlines()[0])


def parse_meminfo(text: str) -> MemoryMetrics:
    values = {}
    for line in text.splitlines():
        key, separator, raw_value = line.partition(':')
        if separator:
            values[key] = int(raw_value.strip().split()[0]) * 1024
    if 'MemTotal' not in values or 'MemAvailable' not in values:
        raise ValueError('MemTotal and MemAvailable are required')
    return MemoryMetrics(values['MemTotal'], values['MemAvailable'])


def read_memory(path: Path = Path('/proc/meminfo')) -> MemoryMetrics:
    return parse_meminfo(path.read_text(encoding='utf-8'))


def read_process_rss(path: Path = Path('/proc/self/status')) -> int:
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) * 1024
    raise ValueError('VmRSS not present')


def read_gpu_percent(paths: Iterable[Path]) -> Optional[float]:
    for path in paths:
        try:
            raw = float(path.read_text(encoding='utf-8').strip())
            return max(0.0, min(100.0, raw / 10.0))
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    return None


def read_max_temperature(paths: Iterable[Path]) -> Optional[float]:
    temperatures = []
    for path in paths:
        try:
            raw = float(path.read_text(encoding='utf-8').strip())
            temperatures.append(raw / 1000.0 if abs(raw) > 500.0 else raw)
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    return max(temperatures) if temperatures else None


def read_thermal_throttle(cooling_devices: Iterable[Path]) -> str:
    states = []
    for device in cooling_devices:
        try:
            device_type = (device / 'type').read_text(encoding='utf-8').strip().lower()
            if 'cpufreq' not in device_type and 'devfreq' not in device_type:
                continue
            states.append(int((device / 'cur_state').read_text(encoding='utf-8').strip()))
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    if not states:
        return 'unknown'
    return 'active' if any(state > 0 for state in states) else 'inactive'
