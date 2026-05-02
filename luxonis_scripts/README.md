# Luxonis scripts

## Install (Ubuntu)

```bash
sudo apt update
sudo apt install -y python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-rtsp-server-1.0 \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly python3-pip python3-venv ffmpeg

cd /path/to/robot-repo-ros2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -U pip
pip install "depthai<3" opencv-python numpy

echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/99-depthai.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Install (openSUSE)

Replace `313` with your `python3` minor version if needed (e.g. `312`).

```bash
sudo zypper install -y python313-gobject typelib-1_0-Gst-1_0 typelib-1_0-GstRtspServer-1_0 \
  libgstrtspserver-1_0-0 gstreamer-plugins-base gstreamer-plugins-good \
  gstreamer-plugins-bad gstreamer-plugins-ugly python313-pip

cd /path/to/robot-repo-ros2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -U pip
pip install "depthai<3" opencv-python numpy

echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/99-depthai.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## RTSP path tags (`rtsp://HOST:8554/<tag>`)

| `--sources` | Stream tags (paths after `:8554/`) |
|-------------|-------------------------------------|
| `ffc` | `ffc0`, `ffc1`, `ffc2`, `ffc3` |
| `oakd` | `oakd_rgb`, `oakd_left`, `oakd_right` |
| `both` | all of the above |

Example: `rtsp://localhost:8554/oakd_rgb`

## Encoder (from repo root, venv on — start before the viewer)

| Goal | Command |
|------|---------|
| FFC 4P only | `python3 luxonis_scripts/luxonis_encode_gstreamer.py --sources ffc` |
| OAK-D only | `python3 luxonis_scripts/luxonis_encode_gstreamer.py --sources oakd` |
| FFC + OAK-D | `python3 luxonis_scripts/luxonis_encode_gstreamer.py --sources both` |
| Device JSON | `python3 luxonis_scripts/luxonis_encode_gstreamer.py --sources both --config luxonis_scripts/luxonis_devices.json` |
| Override MxIds | `... --sources both --ffc-mxid <id> --oakd-mxid <id>` |
| Lower bandwidth | `... --sources both --bitrate 500000` |
| FPS | `... --sources both --fps 24` |
| Slow USB / discovery | `... --discovery-attempts 20 --discovery-delay 3` |

## Viewer (match `--sources` to the running encoder)

| Goal | Command |
|------|---------|
| Local grid, same as encoder | `python3 luxonis_scripts/luxonis_viewer.py --host localhost --sources both` |
| Remote host | `python3 luxonis_scripts/luxonis_viewer.py --host 192.168.1.10 --sources both` |
| FFC tiles only | `python3 luxonis_scripts/luxonis_viewer.py --host localhost --sources ffc` |
| OAK-D tiles only | `python3 luxonis_scripts/luxonis_viewer.py --host localhost --sources oakd` |
| Decode via FFmpeg | `... --rtsp-backend ffmpeg` |
| Force GStreamer | `... --rtsp-backend gstreamer` |
| Upside-down cameras | `... --rotate 180` |
| Bigger tiles | `... --cell-width 960 --cell-height 540` |
| YOLO | `... --yolo` (optional: `--model weights.pt --conf 0.5 --stride 2 --imgsz 640 --half --yolo-rgb-only`; needs `pip install ultralytics torch`) |

Wrapper (same args as `luxonis_viewer.py` after `--`):

```bash
./luxonis_scripts/open_gstreamer_stream.sh
./luxonis_scripts/open_gstreamer_stream.sh 192.168.1.10 -- --sources ffc --rtsp-backend ffmpeg
```

## Viewer keyboard shortcuts

| Key | Action |
|-----|--------|
| `q` | Quit |
| `f` | Fullscreen: zoom focused stream (uses “next” stream from `[` / `]` / arrows — see bar). `f` or **Esc** returns to grid. |
| **Esc** | Leave fullscreen, back to grid |
| `[` or **←** or **↑** | Previous stream (also switches which stream **f** will fullscreen) |
| `]` or **→** or **↓** or **Tab** | Next stream |
| `1`–`9` | Jump to stream by index (1 = first) |
| `l` | Toggle stream labels on/off |
| `y` | Toggle YOLO on/off (loads model on first enable if not started with `--yolo`) |
