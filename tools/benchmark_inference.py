#!/usr/bin/env python3
"""Benchmark a pinned YOLOv5 PyTorch or TensorRT backend."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics
import sys
import time

def temperature_c() -> float | None:
    values = []
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            value = float(path.read_text().strip())
            values.append(value / 1000.0 if value > 1000.0 else value)
        except (OSError, ValueError):
            continue
    return max(values) if values else None


def gpu_load_percent() -> float | None:
    path = Path('/sys/devices/gpu.0/load')
    try:
        return float(path.read_text().strip()) / 10.0
    except (OSError, ValueError):
        return None


def percentile(values: list[float], ratio: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * ratio))]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['pytorch', 'tensorrt'], default='pytorch')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--weights', default='models/yolov5s.pt')
    parser.add_argument('--engine', default='models/yolov5s_fp16.engine')
    parser.add_argument('--warmup', type=int, default=50)
    parser.add_argument('--runs', type=int, default=500)
    parser.add_argument('--output-dir', default='artifacts/benchmark')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.warmup < 0 or args.runs < 1:
        raise ValueError('warmup must be non-negative and runs must be positive')
    if args.dry_run:
        print(json.dumps(vars(args), indent=2))
        return

    import psutil

    repo = Path('vendor/yolov5').resolve()
    sys.path.insert(0, str(repo))
    import torch
    from models.common import DetectMultiBackend

    model_path = Path(args.engine if args.backend == 'tensorrt' else args.weights)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    device = torch.device(args.device)
    model = DetectMultiBackend(str(model_path), device=device, fp16=args.backend == 'tensorrt')
    tensor = torch.zeros((1, 3, 640, 640), device=device)
    if args.backend == 'tensorrt':
        tensor = tensor.half()

    def synchronize() -> None:
        if device.type == 'cuda':
            torch.cuda.synchronize(device)

    with torch.no_grad():
        for _ in range(args.warmup):
            model(tensor)
        synchronize()
        latencies = []
        process = psutil.Process()
        process.cpu_percent(None)
        for _ in range(args.runs):
            started = time.perf_counter()
            model(tensor)
            synchronize()
            latencies.append((time.perf_counter() - started) * 1000.0)

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    mean_ms = statistics.mean(latencies)
    result = {
        'backend': args.backend,
        'device': args.device,
        'warmup_runs': args.warmup,
        'measured_runs': args.runs,
        'fps': 1000.0 / mean_ms,
        'mean_inference_ms': mean_ms,
        'p95_inference_ms': percentile(latencies, 0.95),
        'cpu_percent': process.cpu_percent(None),
        'gpu_percent': gpu_load_percent(),
        'rss_mb': process.memory_info().rss / 1024.0 / 1024.0,
        'temperature_c': temperature_c(),
    }
    (output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    with (output / 'latencies.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['run', 'inference_ms'])
        writer.writerows(enumerate(latencies, start=1))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
