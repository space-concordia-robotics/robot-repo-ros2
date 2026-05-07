# Luxonis Scripts

We have two Luxonis cameras, and OAK-FFC-4P (4 directional cameras) and an OAK-D-PRO (depth camera)

We are running [this depth model on the PRO](https://models.luxonis.com/luxonis/crestereo/4729a8bd-54df-467a-92ca-a8a5e70b52ab) and a custom trained YOLO model for the object detection

## Setup

The current setup requires depthAI v3, which can be installed with:

**requires python3 (can be installed with `sudo apt install python3`)**
```
git clone https://github.com/luxonis/depthai-core.git && cd depthai-core

python3 -m venv venv

source venv/bin/activate (use activate.fish if using a fish terminal)

# Installs library and requirements
python3 examples/python/install_requirements.py

# Additional libraries needed for depth model
pip install depthai-nodes
pip install opencv-python
```

### Set usb rules

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```

### How to run the program

The most reliable way that I've found to connect to devices from PoE is to connnect directly to their IP.  For the black mydlink router,
the most common ip addressive ive found have been 192.168.0.100, and 192.168.0.102.  Switching depending on how the devices are connected (just FFC vs FFC+OAKD, just OAKD)

To run the camera:
```
python3 main.py --mode [PipelineType] -d [MXID or IP](optional)
```

The available modes are:
mode       -> bitrate(b/ps)   -> fps
------------------------------
ffc_all    -> 2000000   -> 30
ffc_front  -> 7000000   -> 30
fc_back    -> 7000000   -> 30
ffc_right  -> 7000000   -> 30
ffc_left,  -> 7000000   -> 30
oakd_all,  -> 1000000   -> 5
oakd_rgb,  -> 7000000   -> 30
oakd_depth -> 1000000   -> 5

### While a pipeline is running

To stop a pipeline run `stop` in the same terminal that ran it

To start a new pipeline, first run `stop`, then run `mode [mode_name]` to switch to the desired mode.

To check bandwidth run `bw`

To continuously check bandwidth run `watch [interval(s)]`

To check status run `status`