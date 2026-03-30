# Orin Nano ML Setup

ML environment setup for [[Jetson Orin]] after completing [[Jetson Orin Nano Setup]].

## Prerequisites

- JetPack 6.2.2 (L4T R36.5.0) installed and running
- CUDA verified (`nvcc --version` shows 12.6)
- Internet connection

## Step 1: Create Virtual Environment

Use `--system-site-packages` so PyTorch can access system CUDA/TensorRT libraries:

```bash
python3 -m venv --system-site-packages ~/venvs/ml
source ~/venvs/ml/bin/activate
```

Add to your `.bashrc` so it activates on login (optional):
```bash
echo 'source ~/venvs/ml/bin/activate' >> ~/.bashrc
```

## Step 2: Install PyTorch

Standard PyTorch wheels won't work — Jetson is ARM (aarch64), not x86. Use NVIDIA's Jetson-specific wheels:

```bash
# Install PyTorch 2.5 for JetPack 6.2 / CUDA 12.6
pip3 install torch torchvision torchaudio --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

Verify:
```bash
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
# Should show version and "True"
```

## Step 3: Install Ultralytics YOLO

Use `[export]` extras to include TensorRT export dependencies:

```bash
pip install ultralytics[export]
```

Test with a quick inference:
```bash
yolo predict model=yolov8n.pt source=https://ultralytics.com/images/bus.jpg
```

## Step 4: Install onnxruntime-gpu (for ONNX export)

PyPI does not have aarch64 binaries for `onnxruntime-gpu` — install the Jetson-specific wheel manually:

```bash
# JetPack 6.x + Python 3.10
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
```

Verify:
```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
```

## Step 5: TensorRT Python Bindings

TensorRT is pre-installed with JetPack. The `--system-site-packages` venv flag (Step 1) should expose it automatically.

> ⚠️ **Do NOT run `pip install tensorrt`** — TensorRT explicitly does not build PyPI wheels for Tegra/ARM systems and will throw `RuntimeError: TensorRT does not currently build wheels for Tegra systems`.

```bash
# Check if accessible in your venv
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

If `import tensorrt` fails, the fix is **not pip** — it means the venv wasn't created with `--system-site-packages`. Recreate it:
```bash
deactivate
python3 -m venv --system-site-packages ~/venvs/mpg-edge
source ~/venvs/mpg-edge/bin/activate
```

Alternatively, install the system TRT Python package via apt:
```bash
sudo apt install python3-libnvinfer python3-libnvinfer-dev
```

## Step 6: Export YOLO to TensorRT (optional)

For faster inference, export to TensorRT engine:

```bash
yolo export model=yolov8n.pt format=engine device=0
```

This creates a `.engine` file optimized for your specific Jetson GPU.

## Troubleshooting

**`pip install tensorrt` fails with "does not build wheels for Tegra systems"**
Don't use pip for TensorRT on Jetson — it's ARM/Tegra and has no PyPI wheel. Use system packages instead:
```bash
sudo apt install python3-libnvinfer python3-libnvinfer-dev
# or recreate venv with --system-site-packages (see Step 1)
```

**`pip3 install` fails with permission error**
```bash
pip3 install --user <package>
# or use a virtual environment
```

**torch.cuda.is_available() returns False**
PyTorch wasn't built for Jetson CUDA. Reinstall from NVIDIA's wheels (Step 1).

**Out of memory during model export**
Close other applications, or use a smaller model (yolov8n instead of yolov8x).

**cuDNN version mismatch (libcudnn.so.8 not found)**
JetPack 6.2 ships cuDNN 9, but older NVIDIA wheels expect cuDNN 8. Try:
```bash
# Use Jetson AI Lab wheels instead of NVIDIA developer wheels
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

## mpg-ai-edge Repo Setup

**GitHub:** [mosscoder/mpg-ai-edge](https://github.com/mosscoder/mpg-ai-edge)

```bash
mkdir -p ~/dev
git clone https://github.com/mosscoder/mpg-ai-edge.git ~/dev/mpg-ai-edge
```

### Why `environment-jetson.yml` Doesn't Work on JetPack 6.2

The repo's conda environment file was likely created for an **older JetPack version** (probably 5.x). It fails on JetPack 6.2 for several reasons:

| Issue | `environment-jetson.yml` | JetPack 6.2 Requires |
|-------|--------------------------|----------------------|
| Python version | 3.8 | 3.10 (wheels only built for 3.10) |
| cuDNN version | cuDNN 8 | cuDNN 9 (breaking change) |
| PyTorch source | PyPI (x86 CPU-only) | Jetson AI Lab (aarch64 + CUDA) |
| Architecture | Assumes x86_64 | ARM64 (aarch64) |

**What changed in JetPack 6.x:**
- NVIDIA upgraded from cuDNN 8 → cuDNN 9 (not backward compatible)
- PyTorch wheels must be compiled specifically for cuDNN 9
- Only Python 3.10 wheels are provided for JetPack 6.x

**Why conda doesn't work well on Jetson:**
- Conda manages its own Python + libraries, which conflicts with JetPack's system CUDA/cuDNN/TensorRT
- Conda's pip integration doesn't reliably use custom index URLs (pulls CPU-only wheels from PyPI)
- The `--system-site-packages` approach works *with* JetPack instead of fighting it

### Recommended Setup: venv (Not Conda)

Use Python's built-in venv with system site packages:

```bash
# Install venv if needed
sudo apt install python3.10-venv -y

# Create venv with access to system CUDA libs
python3 -m venv --system-site-packages ~/venvs/mpg-edge
source ~/venvs/mpg-edge/bin/activate

# Install PyTorch from Jetson AI Lab
pip install torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# Install dependencies
pip install -r requirements-jetson.txt

# Verify CUDA
python -c "import torch; print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
# Should print: PyTorch 2.8.0, CUDA: True
```

**`requirements-jetson.txt`** contents (reconstructed 2026-03-30 — was lost when previous board was fried, not committed to repo):

```
numpy<2
Pillow>=9.1  # venv installs 12.1.1, shadowing system 9.0.1 — this is fine
transformers
opencv-python
pyserial
pynmea2
```

> ⚠️ `torch`, `torchvision`, `ultralytics[export]`, and `onnxruntime-gpu` are installed separately with pinned versions — do not add them here.

### Alternative: Docker Containers

NVIDIA's [jetson-containers](https://github.com/dusty-nv/jetson-containers) provides pre-built Docker images:

```bash
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
jetson-containers run $(autotag l4t-pytorch)
```

This is the most reproducible approach for team environments.

## Update Log

**2026-02-02: Switched from Conda to venv**
- Conda fights with JetPack's system libraries — venv with `--system-site-packages` works better
- Removed Miniforge, using simple Python venv instead
- Documented why `environment-jetson.yml` fails (JetPack 5.x vs 6.x differences)

**2026-02-02: PyTorch CUDA working ✓**
- Jetson AI Lab domain is `.io` not `.dev` (pypi.jetson-ai-lab.io)
- PyTorch 2.9.1 requires cuDSS (not in JetPack 6.2.1) — use 2.8.0 instead
- NumPy must be <2 (PyTorch 2.8.0 compiled against NumPy 1.x)
- Verified: `PyTorch 2.8.0, CUDA: True`

**2026-02-02: DINOv2 inference working ✓**
- System Pillow 9.0.1 lacks `PIL.Image.Resampling` — need `Pillow>=9.1` in venv
- System scipy expects NumPy <1.25 — need `scipy>=1.11` in venv to avoid warning
- Ran `00_jetson_offline_inference.py` successfully (768-dim features on CUDA)
- Created [[requirements-jetson.txt]] for reproducible venv setup

## Resources

- [NVIDIA PyTorch for Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [Jetson AI Lab PyTorch Wheels](https://pypi.jetson-ai-lab.io/jp6/cu126) ⚠️ Domain is `.io` not `.dev`
- [NinjaLABO PyTorch Guide](https://ninjalabo.ai/blogs/jetson_pytorch.html)
- [NVIDIA Developer Forums - PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
- [NVIDIA Developer Forums - JetPack 6.2 Install Thread](https://forums.developer.nvidia.com/t/install-pytorch-for-cuda-12-6-jetpack-6-2/348456)

## Links

[[Jetson Orin Nano]] | [[Jetson Orin]] | [[Jetson Orin Nano Status]] | [[TensorRT]] | [[Edge AI MOC]] | [[mpg-robotanist]]
