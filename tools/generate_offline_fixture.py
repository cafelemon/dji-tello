#!/usr/bin/env python3
"""Generate a deterministic moving-person video from the pinned YOLOv5 fixture."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', default='vendor/yolov5/data/images/zidane.jpg')
    parser.add_argument('--output', default='artifacts/fixtures/person_motion.mp4')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--fps', type=float, default=30.0)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.fps <= 0.0:
        raise ValueError('duration and fps must be positive')
    image = cv2.imread(args.source)
    if image is None:
        raise RuntimeError(f'cannot read fixture source: {args.source}')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = 640, 480
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError(f'cannot create fixture video: {output}')
    source_height, source_width = image.shape[:2]
    frame_count = max(1, round(args.duration * args.fps))
    for index in range(frame_count):
        phase = 2.0 * math.pi * index / frame_count
        scale = 0.72 + 0.08 * math.sin(phase * 2.0)
        base_scale = max(width / source_width, height / source_height) * scale
        matrix = np.array(
            [
                [base_scale, 0.0, width / 2.0 - source_width * base_scale / 2.0 + 35.0 * math.sin(phase)],
                [0.0, base_scale, height / 2.0 - source_height * base_scale / 2.0 + 18.0 * math.cos(phase)],
            ],
            dtype=np.float32,
        )
        frame = cv2.warpAffine(image, matrix, (width, height), borderMode=cv2.BORDER_REFLECT)
        writer.write(frame)
    writer.release()
    print(output)


if __name__ == '__main__':
    main()
