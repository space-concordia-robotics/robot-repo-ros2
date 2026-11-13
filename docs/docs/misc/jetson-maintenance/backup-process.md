# Jetson Backup & Restoration

## Backup Process

The Jetson should be periodically backed up in order to allow for recovery in case of

- damage to the hardware
- disk failure
- messing something up on the Jetson

The backup process involves periodically taking a disk image of the Jetson and saving it somewhere.

Steps:

1. Remove the Jetson from the rover, and power it using an external power supply (can be found in the TX2 box)
2. ssh into the Jetson and clean up any unnecessary stuff that isn't super important to back up.
    1. run `apt-fast autodelete`
    2. run `apt-fast clean`
    3. delete any `build`, `logs`, and `install` directories for ROS workspaces
    4. delete `~/.ros/log` as it contains some HUGE files, which are not necessary.
    5. empty the trash. this can be done with trash-cli using `trash-list`, `trash-rm`, and `trash-empty`.
    6. run `sudo ncdu -x /` and look for anything that
3. Fully shut down the Jetson and unplug it.
4. Unscrew & remove the nvme ssd from the Jetson
5. Plug the nvme ssd into a laptop using the nvme adapter puck
6. run the following command (here this assumes the disk is `/dev/sda`. this can be checked using `fdisk -l`):
    ```bash
    sudo dd if=/dev/sda bs=4M | zstd -o "jetson-$(date +'%Y-%m-%d').img.zst" -11
    ```
7. Put the disk back in the Jetson

## Restoration

To restore the Jetson back to a previous disk image, do the following:

1. Remove the Jetson from the rover
2. Plug the nvme ssd into a laptop using the nvme adapter puck
3. run the following command (here this assumes the disk is `/dev/sda`. this can be checked using `fdisk -l`):
    ```bash
    zstd -d 'jetson-[date].img.zst' --stdout | dd of=/dev/sda bs=4M
    ```
4. Put the disk back in the Jetson

This should restore the disk image.
