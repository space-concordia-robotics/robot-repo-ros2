# CAN Bus

We use CAN Bus to control & talk to the wheels and several other devices.

The Nvidia Jetson automatically configures & starts the CAN bus network on startup.

In order to load the appropriate kernel modules (`can`, `can_raw`, and `mttcan`), it has:

```bash title="/etc/modules-load.d/can.conf"
## load linux kernel modules automatically

# linux socketcan
can
can_raw

# nvidia can driver
mttcan
```

Then, to start up the network, there are the following three files:

```systemd title="/etc/systemd/network/80-can.network"
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=100ms
```

```systemd title="/etc/systemd/network/80-can.link"
[Match]
Type=can

[Link]
TransmitQueueLength=512
```

```systemd title="/etc/udev/rules.d/80-can.rules"
SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add|change", ATTR{tx_queue_len}="512"
```

After adding or changing the 3rd file, you must run:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=net --action=change
```
