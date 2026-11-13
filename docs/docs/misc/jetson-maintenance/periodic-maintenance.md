# Periodic Maintenance

Periodically, the Jetson should have any package updates installed to it.
This can be done with the following commands:

```bash
apt-fast update
apt-fast upgrade
```

!!! warning inline end

    Sometimes it can be useful to run `apt-fast dist-upgrade`, however **be careful** with it, as it can cause packages to be removed.

To clean up any leftover files or packages which are no longer needed, run the following commands:

```bash
apt-fast autodelete
apt-fast clean
```
