# TODO

This file is for things that need to be moved into a better location later.

## identifying `/dev/` devices

Options:

### watching `udevadm monitor`:

1. run
   ```bash
   sudo udevadm monitor -u
   ```
2. un-plug & re-plug device

### `udevadm info`:

run the following command:

```bash
udevadm info -q all -a -n /dev/ttyACM0 | grep -i 'manufacturer\|product'
```
