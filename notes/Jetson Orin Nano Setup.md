# Jetson Orin Nano

Setup checklist for the [[mpg-robotanist]] project.

## Hardware Needed

- [ ] Jetson Orin Nano Developer Kit
- [ ] microSD card (64GB+, fast read/write recommended)
- [ ] SD card reader
- [ ] Power supply via **barrel jack** (5.5×2.5mm, center positive, 9–20V) — the official dev kit PSU is 19V/2.37A
   > ⚠️ **USB-C is data/flash only — never power input.** Feeding power through USB-C will fry the board.
- [ ] Monitor + **DisplayPort** cable (Orin dev kit uses DP, not HDMI — get a DP→HDMI adapter if needed)
- [ ] USB keyboard and mouse (for initial setup)

## Step 1: Flash JetPack SD Card

**On your computer (any OS):**

1. Download JetPack 6.2.1 SD image (used as base for 6.2.2):
   - Go to https://developer.nvidia.com/embedded/jetpack-sdk-621
   - Select "SD Card Image" for Orin Nano Developer Kit
   - ~18GB download
   - *Note: No standalone 6.2.2 SD image exists — flash 6.2.1, then APT upgrade below*

2. **Unzip the downloaded file first** — NVIDIA delivers the image as a `.zip`. Extract it to get `sd-blob.img` before flashing.
   > ⚠️ If you feed the `.zip` directly to Etcher you'll get "Error starting flasher sidecar process" — this is Etcher failing to decompress, not a permissions issue.

3. Flash with [Balena Etcher](https://etcher.balena.io/):
   - Open Etcher
   - Select the unzipped `sd-blob.img`
   - Select your SD card
   - Click "Flash!" — ~15-20 minutes

## Step 2: First Boot

1. Insert flashed SD card into Nano (slot on module underside)
2. Connect monitor, keyboard, mouse
3. Connect power — it boots automatically
4. Walk through Ubuntu setup wizard:
   - Language, timezone
   - **Connect to WiFi** — do this during the wizard, all following steps require network
   - Create user account (remember this password!)
   - Let it finish initial configuration

If WiFi wasn't configured during setup, connect via CLI before proceeding:
```bash
nmcli dev wifi list
nmcli dev wifi connect "YourNetwork" password "YourPassword"
```

## Step 3: SSH Access (enabled by default)

SSH is typically already running after the Ubuntu wizard. From your other computer on the same network:
```bash
ssh mpg-robodog@mpg-robodog.local
```

The `.local` hostname uses mDNS — no need to look up the IP address. Works reliably on any Mac on the same WiFi network. Once connected, you can disconnect the monitor and work headlessly.

## Step 4: Install tmux

Install before any long-running apt commands — protects against SSH disconnects losing your session:

```bash
sudo apt install tmux -y
tmux new -s jetson
```

If you get disconnected at any point, re-attach with:
```bash
tmux attach -t jetson
```

## Step 5: Upgrade to JetPack 6.2.2

No standalone 6.2.2 SD image exists — upgrade via APT after first boot:

```bash
# Edit apt sources to point to r36.5
sudo vi /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
# Change all repo version references to r36.5, then save

sudo apt update
sudo apt dist-upgrade
sudo reboot
```

## Step 6: Verify Installation

```bash
# Check JetPack version (should show R36 REVISION: 5.0 for JetPack 6.2.2)
cat /etc/nv_tegra_release

# Add CUDA to PATH (not in PATH by default)
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc && source ~/.bashrc

# Verify CUDA — should show CUDA 12.6
nvcc --version

# Verify TensorRT
dpkg -l | grep tensorrt
# Should show tensorrt 10.3.0

# Check GPU
sudo tegrastats
# Ctrl+C to stop — should show GPU temp, RAM, power draw
```

## Next Steps

After base setup, continue to [[Orin Nano ML Setup]] for:
- Python environment
- PyTorch for Jetson
- Ultralytics YOLO
- TensorRT Python bindings

## Optional: Set Performance Mode

JetPack 6.2+ defaults to 25W (MAXN SUPER) mode. Check current mode:
```bash
sudo nvpmodel -q
```

Change if needed:
```bash
# 25W max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# 15W power-saving
sudo nvpmodel -m 1
```

## Troubleshooting

**`nvcc: command not found`**
CUDA isn't in PATH by default on JetPack 6.2.2 — this is expected. Fix (also in Step 7):
```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc && source ~/.bashrc
```

**Snap store errors**
Snap is unreliable on Jetson ARM. Use apt instead:
```bash
sudo apt install chromium-browser  # instead of snap
```

**dpkg lock error**
Another update is running. Check with `ps aux | grep apt` and wait for it to finish.

**Can't find microSD slot**
It's on the underside of the carrier board, near the USB-C power port. Insert with contacts facing up.

## Resources

- [NVIDIA Getting Started Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit#intro) — official docs with labeled board diagrams
- [DroneBot Workshop - Jetson Orin Nano Setup](https://dronebotworkshop.com/jetson-orin-nano/) — 48 min video tutorial covering microSD and SSD setup with clear visuals
- [NVIDIA Getting Started Video](https://www.youtube.com/watch?v=VWdJ4BCtam8) — official walkthrough
- [JetsonHacks YouTube](https://www.youtube.com/@JetsonHacks/videos) — excellent channel for Jetson tutorials, tips, and project ideas
- [Jetson AI Fundamentals](https://www.youtube.com/watch?v=MnaohuzEuhA) — NVIDIA tutorial
- [JetPack 6.2.2 SDK Page](https://developer.nvidia.com/embedded/jetpack-sdk-622) — latest release notes
- [JetPack 6.2.1 Download](https://developer.nvidia.com/embedded/jetpack-sdk-621) — SD card image (flash this, then APT upgrade to 6.2.2)

## Links

[[Jetson Orin]] | [[JetPack]] | [[mpg-robotanist]] | [[TensorRT]] | [[Edge AI MOC]] | [[2026-01-28]]
