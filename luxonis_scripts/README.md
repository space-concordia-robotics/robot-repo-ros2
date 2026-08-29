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

To run the camera:
```
python3 main.py --mode [PipelineType] -d [MXID or IP]
```

IPs* OAK-D Pro `10.240.0.67`, FFC PoE `10.240.0.69`.

**Modes**

| mode | bitrate | fps | RTSP paths |
|------|---------|-----|------------|
| ffc_all | 2 Mbps | 30 | `/FRONT` `/RIGHT` `/LEFT` `/BACK` |
| ffc_front / ffc_back / ffc_right / ffc_left | 7 Mbps | 30 | one of the above |
| oakd_rgb | 7 Mbps | 30 | `/RGB` |
| oakd_yolo | 4 Mbps | 15 | `/RGB` + terminal `detections` |
| all_cams | FFC 2 Mbps + RGB 7 Mbps | 30 | all five paths |

## REPL commands

To stop a pipeline run `stop` in the same terminal that ran it

To start a new pipeline, first run `stop`, then run `mode [mode_name]` to switch to the desired mode.

To check bandwidth run `bw`
To continuously check bandwidth run `watch [interval(s)]`

To check status run `status`


## main.py examples (To launch all cameras)

  python3 main.py --mode oakd_rgb -d 10.240.0.50
```
  > mode ffc front,right                   # OAK keeps running; FFC starts with 2 cams
  > mode ffc front,right,left              # FFC restarts with 3 cams; OAK untouched
  > mode ffc front 10.240.0.40             # FFC = front only, on explicit device
  > mode ffc_all                            # FFC restarts with all 4
  > stop ffc                               # OAK keeps running
  > stop                                   # stops both
```
  Valid ffc cameras: front, right, left, back (case-insensitive).

## launch_cameras.sh examples

```
  bash launch_cameras.sh ffc front right       # 2 ffplay windows
  bash launch_cameras.sh ffc front left back   # 3 windows
  bash launch_cameras.sh ffc_front             # 1 (alias still works)
  bash launch_cameras.sh oakd_rgb              # OAK
  bash launch_cameras.sh all                    # FFC 4-up + OAK RGB
```