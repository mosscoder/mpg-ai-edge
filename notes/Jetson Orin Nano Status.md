# Jetson Orin Nano — Status

**Last updated:** 2026-03-30
**Hardware:** NVIDIA Jetson Orin Nano 8GB Developer Kit (replacement unit — previous unit damaged 2026-03-24)
**Software:** JetPack 6.2.2 / L4T R36.5.0 (CUDA 12.6, TensorRT 10.3.0)
**Hostname:** `mpg-robodog`
**SSH:** `ssh mpg-robodog@mpg-robodog.local`

## Current State: ML Environment Ready ✅

| Component | Status | Notes |
|-----------|--------|-------|
| JetPack 6.2.2 | ✅ | L4T R36.5.0, CUDA 12.6, TensorRT 10.3.0 |
| Hardware verified | ✅ | 8GB RAM, 6-core CPU, GPU ~42°C idle |
| CUDA in PATH | ✅ | Added to `~/.bashrc` |
| tmux | ✅ | Session: `jetson` |
| SSH access | ✅ | `mpg-robodog@mpg-robodog.local` |
| Python venv | ✅ | `~/venvs/mpg-edge` |
| PyTorch 2.8.0 | ✅ CUDA | Jetson AI Lab wheels |
| Ultralytics 8.4.31 | ✅ | `pip install ultralytics[export]` |
| onnxruntime-gpu 1.23.0 | ✅ | aarch64 wheel from Ultralytics assets |
| TensorRT 10.3.0 | ✅ | System install via `--system-site-packages` |
| YOLO inference | ✅ | YOLOv8n: 187ms, CUDA:0 confirmed |
| DINOv2 inference | ✅ | 768-dim features on CUDA, `00_jetson_offline_inference.py` |
| mpg-ai-edge repo | ✅ | Cloned to `~/dev/mpg-ai-edge` |

## Quick Start

```bash
ssh mpg-robodog@mpg-robodog.local
tmux attach -t jetson
source ~/venvs/mpg-edge/bin/activate
python3 -c "import torch; print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
```

## Repo

**mpg-ai-edge:** [mosscoder/mpg-ai-edge](https://github.com/mosscoder/mpg-ai-edge)

```bash
mkdir -p ~/dev
git clone https://github.com/mosscoder/mpg-ai-edge.git ~/dev/mpg-ai-edge
```

## History

| Date | Event |
|------|-------|
| 2026-03-24 | Previous unit damaged — 12V fed into USB-C port (data only), burn mark near barrel jack |
| 2026-03-30 | Replacement unit flashed, JetPack 6.2.2 verified, full ML stack installed, YOLO + DINOv2 inference confirmed |

## Links

[[Jetson Orin Nano]] | [[Jetson Orin Nano Setup]] | [[Orin Nano ML Setup]] | [[Jetson Orin Nano Specs]] | [[mpg-robotanist]]
