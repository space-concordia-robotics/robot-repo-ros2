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
