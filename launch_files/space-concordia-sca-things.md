# CSA Stuff

## Base Station

- Password for computer: `scrb2025`
- Password for wifi access point: `ilovespace`

## Wireless Comms
Url for base station antenna: http://10.240.0.21
Url for rover antenna: http://10.240.0.22

Wireless tab: Output power
- Start at -4dBm
- Increase slowly
- Keep signal strength around -40dBm (see main tab on rover antenna)

## Rover
Preparing rover for driving:
- `./launch-rover`: starts rover
- `./launch-cameras`: launches camera windows - Run this on base station
- `ros2 run joy joy_node`: runs the joystick - Run this on base station

Foxglove:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

If the rover is not connecting:
1. See if you can ping the base station antenna:
  ```bash
  sudo ping -f 10.240.0.21
  ```
2. See if you can ping the rover antenna:
  ```bash
  sudo ping -f 10.240.0.22
  ```
3. See if you can ping the rover:
  ```bash
  sudo ping -f 10.240.0.10
  ```

Listing topics on the base station:
```bash
ros2 topic list
```

**Note**: The first time this is run it will not list the topics, run it a second time to list them.

### GPS
On base station run
```bash
ros2 topic echo /gps_node/fix
```

### Aruco
On base station run
```bash
ros2 topic echo /tf
```

## Stopping rover
- `./stop-rover`: stops rover

## Cameras

- list cameras:
  ```
  for i in `seq 0 10`; do echo "/dev/video$i"; v4l2-ctl -d /dev/video$i --list-formats-ext; done
  ```
- Modify `./rtsp_cameras.yaml`
- Modify `ardu-cam.py`
