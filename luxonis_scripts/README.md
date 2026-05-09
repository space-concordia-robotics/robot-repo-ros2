# Luxonis Scripts

We have two Luxonis cameras, and OAK-FFC-4P (4 directional cameras) and an OAK-D-PRO (depth camera)

We are running [this depth model on the PRO](https://models.luxonis.com/luxonis/crestereo/4729a8bd-54df-467a-92ca-a8a5e70b52ab) and a custom trained YOLO model for the object detection

## Setup

The current setup requires depthAI v3, which can be installed with:

**requires python3 (can be installed with `sudo apt install python3`)**
```
source venv/bin/activate (use activate.fish if using a fish terminal)

# Installs library and requirements
pip install requirements.txt
```

### Set usb rules

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```

### How to run the program

The most reliable way that I've found to connect to devices from PoE is to c!onnnect directly to their IP.  For the black mydlink router,
the OAK-D-PRO IP address has been manually set to 10.240.0.67 on the basestation router.  Switching depending on how the devices are connected (just FFC vs FFC+OAKD, just OAKD)

To run the camera:
```
python3 main.py --mode [PipelineType] -d [MXID or IP]
```

The available modes are:
mode       ->   bitrate(b/ps)   -> fps  ->  link    
--------------------------------------
ffc_all    ->   2000000         -> 30   ->  N/A
ffc_front  ->   7000000         -> 30   ->  rtsp://localhost:8554/FRONT
fc_back    ->   7000000         -> 30   ->  rtsp://localhost:8554/BACK
ffc_right  ->   7000000         -> 30   ->  rtsp://localhost:8554/RIGHT
ffc_left,  ->   7000000         -> 30   ->  rtsp://localhost:8554/LEFT
oakd_rgb,  ->   7000000         -> 30   ->  rtsp://localhost:8554/RGB
oakd_yolo  ->   4000000         -> 15   ->  rtsp://localhost:8554/RGB

To see the currently live cameras in the pipeline go to http://localhost:8082/

### While a pipeline is running

To stop a pipeline run `stop` in the same terminal that ran it

To start a new pipeline, first run `stop`, then run `mode [mode_name]` to switch to the desired mode.

To check bandwidth run `bw`
To continuously check bandwidth run `watch [interval(s)]`

To check status run `status`

To check for aruco tags run `aruco`
To check continuously for aruco tags run `watch_aruco`

### USB CAMERAS (Backup only)

```
sudo zypper install \
          gstreamer-plugins-good \
          gstreamer-plugins-bad \
          gstreamer-plugins-ugly \
          gstreamer-plugins-libav \
          v4l-utils
```

python3 usb_cameras.py \\
    --camera "FRONT=/dev/video0,1280x720,30,mjpeg" \\
    --camera "RIGHT=/dev/video2,960x600,10,yuyv" \\
    --camera "LEFT=/dev/video4,960x600,10,yuyv" \\
    --camera "BACK=/dev/video6,960x600,10,yuyv"