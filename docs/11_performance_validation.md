# 11 性能验证规范

## 推理基准

```bash
python3 tools/benchmark_inference.py \
  --backend pytorch --device cpu --warmup 50 --runs 500 \
  --output-dir artifacts/benchmark/cpu
```

Jetson 分别执行 PyTorch CUDA 和 TensorRT FP16。每组固定预热 50 次、测量 500 次，保存 `summary.json` 与 `latencies.csv`，记录 FPS、平均/P95 推理耗时、CPU、GPU、RSS 和温度。

```bash
python3 tools/benchmark_inference.py --backend pytorch --device cuda:0 \
  --warmup 50 --runs 500 --output-dir artifacts/benchmark/cuda
python3 tools/benchmark_inference.py --backend tensorrt --device cuda:0 \
  --warmup 50 --runs 500 --output-dir artifacts/benchmark/tensorrt-fp16
```

640×640 输入目标为不低于 15 FPS、P95 推理耗时不高于 150ms。工具始终保存真实结果；未达到目标时保留证据并进入优化，不修改或舍弃测量数据。

## 本机短时端到端稳定性

```bash
tools/run_offline_soak.sh 120 artifacts/offline-short-120
```

通过标准：

- 容器限制为最多 2 核 CPU、4GB 内存；
- 进程完整运行 120 秒，持续发布标注图像和可见目标状态；
- 诊断错误数为 0；
- 使用前 20% 预热后的基准窗口与末段 20% 比较，末段 RSS 中位数不超过基准 15%；
- 末段 RSS 采样正增长比例低于 90%，不存在持续单调增长。

证据目录属于运行产物，不进入 Git。验收状态只在 `docs/04_acceptance_checklist.md` 更新；目标平台延长运行按 `docs/08_hardware_test.md` 单独记录。
