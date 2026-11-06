# Jetson Setup

## Initial Setup

1. Follow official installation instructions
2. Set up user account with the username set to `scrb` and the password set to the known shared password
3. Configure the static IP address
    1. Open a terminal
    2. run `#!shell nm-connection-editor`
    3. Select the wired connection
    4. Open the "IPv4 Settings" tab
    5. Change the "Method" from "Automatic (DHCP)" to "Manual"
    6. Add a static IP address with the following settings:
        - Address: `10.240.0.10`
        - Netmask: `8`
        - Gateway: `10.240.0.1`
    7. Save and exit
4. Enable max performance mode
    1. Click NVIDIA icon in the top-right corner
    2. Select "Power node"
    3. Choose "MAXN" (or "MAXN Super", if available)
    4. Reboot when prompted
5. SSH into the device (`ssh scrb@10.240.0.10`)
   TODO: note to use `ssh-copy-id`

## Debloating

1. Disable the graphical target:

    ```shell
    sudo systemctl set-default multi-user.target
    ```

2. Reboot
3. SSH into the device
4. Remove snaps
    1. Disable snap systemd units

        ```shell
        sudo systemctl disable snapd.service \
                               snapd.socket \
                               snapd.apparmor.service \
                               snapd.autoimport.service \
                               snapd.core-fixup.service \
                               snapd.failure.service \
                               snapd.recovery-chooser-trigger.service \
                               snapd.seeded.service \
                               snapd.snap-repair.service \
                               snapd.system-shutdown.service \
                               snapd.mounts-pre.target \
                               snapd.mounts.target \
                               snapd.snap-repair.timer
        ```

    2. List all the snaps:

        ```console
        $ snap list
        Name               Version                         Rev    Tracking       Publisher   Notes
        bare               1.0                             5      latest/stable  canonical✓  base
        core24             20260410                        1644   latest/stable  canonical✓  base
        firefox            154.0-1                         8762   latest/stable  mozilla✓    -
        gnome-46-2404      0+git.f1cd5fa-sdk0+git.ca9c59c  154    latest/stable  canonical✓  -
        gtk-common-themes  0.1-81-g442e511                 1535   latest/stable  canonical✓  -
        mesa-2404          25.2.8-snap288                  1836   latest/stable  canonical✓  -
        snapd              2.76.2                          27709  latest/stable  canonical✓  snapd
        thunderbird        154.0-2                         1229   latest/stable  canonical✓  -
        ```

    3. Remove all of them (note, the list of snaps installed may be different):

        ```shell
        sudo snap remove --purge firefox thunderbird gnome-46-2404 gtk-common-themes mesa-2404 core24 bare
        sudo snap remove --purge snapd
        ```

    4. Uninstall snap packages

        ```shell
        sudo apt remove --purge snapd firefox thunderbird
        ```

    5. Remove snap directories

        ```shell { .annotate }
        sudo rm /snap /var/snap /var/cache/snapd/ /var/lib/snapd/ /root/snap -rf # (1)!
        rm ~/.snap/ ~/snap/ -rf
        ```

        1. `-rf` is used *after* everything else, because it prevents you from accidentally pressing enter with a partially typed out command and then deleting
           important stuff

    6. Block snaps in the hosts file

        ```shell
        echo "127.0.0.1 api.snapcraft.io" | sudo tee -a /etc/hosts >/dev/null
        ```

5. Remove desktop packages & de-bloat

    ```shell
    sudo apt remove --purge apport apport-core-dump-handler apport-gtk \
                            apport-symptoms baobab bluez-cups colord \
                            colord-data cups cups-browsed cups-bsd cups-client \
                            cups-common cups-core-drivers cups-daemon cups-filters \
                            cups-filters-core-drivers cups-ipp-utils cups-pk-helper \
                            cups-ppdc cups-server-common espeak-ng-data \
                            evince-common evolution-data-server-common file-roller \
                            fonts-droid-fallback fonts-liberation-sans-narrow \
                            fonts-noto-cjk fonts-noto-color-emoji fonts-noto-core \
                            fonts-noto-mono gir1.2-gdm-1.0 gir1.2-gmenu-3.0 \
                            gir1.2-gnomeautoar-0.1 gir1.2-javascriptcoregtk-4.1 \
                            gir1.2-javascriptcoregtk-6.0 gir1.2-totemplparser-1.0 \
                            gir1.2-webkit2-4.1 gir1.2-webkit-6.0 \
                            gnome-accessibility-themes gnome-bluetooth-3-common \
                            gnome-bluetooth-sendto gnome-calculator gnome-calendar \
                            gnome-characters gnome-clocks gnome-control-center \
                            gnome-control-center-data gnome-control-center-faces \
                            gnome-desktop3-data gnome-disk-utility gnome-font-viewer \
                            gnome-initial-setup gnome-keyring gnome-keyring-pkcs11 \
                            gnome-logs gnome-menus gnome-online-accounts \
                            gnome-power-manager gnome-remote-desktop gnome-screenshot \
                            gnome-session-bin gnome-session-canberra \
                            gnome-session-common gnome-settings-daemon \
                            gnome-settings-daemon-common gnome-shell \
                            gnome-shell-common gnome-shell-extension-appindicator \
                            gnome-shell-extension-desktop-icons-ng \
                            gnome-shell-extension-ubuntu-dock \
                            gnome-shell-extension-ubuntu-tiling-assistant \
                            gnome-snapshot gnome-software gnome-software-common \
                            gnome-startup-applications gnome-system-monitor \
                            gnome-terminal gnome-terminal-data gnome-text-editor \
                            gnome-themes-extra gnome-themes-extra-data \
                            gnome-user-docs ibus ibus-data ibus-gtk ibus-gtk3 \
                            ibus-gtk4 ibus-table language-selector-gnome \
                            libcupsfilters2-common libcupsfilters2t64 \
                            libcupsimage2t64 libcupsimage2t64 libespeak-ng1 \
                            libevdocument3-4t64 libevview3-3t64 libgdm1 libglib2.0-doc \
                            libgnome-autoar-0-0 libgnomekbd8 libgnomekbd-common \
                            libgnome-menu-3-0 libgtk2.0-0t64 libgtk2.0-bin \
                            libgtk2.0-common libgtk3-perl libgtk-4-1 libgtk-4-bin \
                            libgtk-4-common libgtk-4-media-gstreamer libgtkmm-4.0-0 \
                            libgtksourceview-5-0 libgtksourceview-5-common libibus-1.0-5 \
                            libjavascriptcoregtk-4.1-0 libjavascriptcoregtk-6.0-1 \
                            libnautilus-extension4 libqt5core5t64 libqt5dbus5t64 \
                            libqt5gui5t64 libqt5network5t64 libqt5opengl5t64 \
                            libqt5test5t64 libqt5widgets5t64 libreoffice-base-core \
                            libreoffice-calc libreoffice-common libreoffice-core \
                            libreoffice-draw libreoffice-gnome libreoffice-gnome \
                            libreoffice-gtk3 libreoffice-impress libreoffice-math \
                            libreoffice-style-colibre libreoffice-style-elementary \
                            libreoffice-style-yaru libreoffice-uiconfig-calc \
                            libreoffice-uiconfig-common libreoffice-uiconfig-draw \
                            libreoffice-uiconfig-impress libreoffice-uiconfig-math \
                            libreoffice-uiconfig-writer libreoffice-writer \
                            libspeechd2 libtotem-plparser18 libtotem-plparser-common \
                            libvte-2.91-0 libvte-2.91-common libwebkit2gtk-4.1-0 \
                            libwebkitgtk-6.0-4 mutter-common mutter-common-bin \
                            nautilus-data nautilus-sendto network-manager-gnome \
                            network-manager-openvpn-gnome network-manager-pptp-gnome \
                            onboard onboard-common openprinting-ppds orca \
                            pinentry-gnome3 printer-driver-brlaser printer-driver-c2esp \
                            printer-driver-foo2zjs printer-driver-foo2zjs-common \
                            printer-driver-hpcups printer-driver-hpcups \
                            printer-driver-m2300w printer-driver-min12xxw \
                            printer-driver-pnm2ppa printer-driver-postscript-hp \
                            printer-driver-postscript-hp printer-driver-ptouch \
                            printer-driver-pxljr printer-driver-sag-gdi \
                            printer-driver-splix python3-cups python3-cupshelpers \
                            python3-ibus-1.0 python3-speechd remmina-common rhythmbox \
                            rhythmbox-data rhythmbox-plugin-alternative-toolbar \
                            rhythmbox-plugins shotwell shotwell-common simple-scan \
                            software-properties-gtk sound-icons speech-dispatcher \
                            speech-dispatcher-audio-plugins speech-dispatcher-espeak-ng \
                            system-config-printer-common system-config-printer-udev \
                            tango-icon-theme tecla totem-common transmission-common \
                            transmission-gtk ubuntu-wallpapers ubuntu-wallpapers \
                            ubuntu-wallpapers-noble ubuntu-wallpapers-noble \
                            update-manager whoopsie whoopsie-preferences xorg-docs-core \
                            xterm yaru-theme-gnome-shell yaru-theme-icon yaru-theme-icon \
                            yaru-theme-sound yaru-theme-sound yelp yelp-xsl
    sudo apt autoremove docker.io docker-doc docker-compose docker-compose-v2 \
                        podman-docker containerd runc
    sudo systemctl mask display-manager.service
    sudo systemctl daemon-reload
    sudo apt-mark auto dbus dbus-bin dbus-daemon dbus-session-bus-common \
                       dbus-system-bus-common dbus-user-session dbus-x11 x11-common \
                       x11proto-dev x11-utils x11-xkb-utils x11-xserver-utils \
                       xorg-sgml-doctools xserver-common xserver-xorg-core \
                       xserver-xorg-input-all xserver-xorg-input-libinput \
                       xserver-xorg-input-wacom xserver-xorg-legacy \
                       xserver-xorg-video-all xserver-xorg-video-amdgpu \
                       xserver-xorg-video-ati xserver-xorg-video-fbdev \
                       xserver-xorg-video-nouveau xserver-xorg-video-radeon \
                       xserver-xorg-video-vesa
    sudo apt-mark auto $(apt-mark showmanual | grep -iE -e '^gir' -e '^glib' -e '^gvfs')
    sudo apt-mark auto $(apt-mark showmanual | grep -iE 'python3?-' | grep -iv 'jetson')
    sudo apt-mark auto $(apt-mark showmanual | grep -iE '^lib' | grep -iv -e 'opencv')
    sudo apt-mark auto $(apt-mark showmanual | sort -u | grep -iE 'x11' -e 'xorg' -e 'xserver')
    sudo apt autoremove
    ```

6. Disable Ubuntu Pro advertisements

    ```shell
    sudo pro config set apt_news=false
    sudo rm /etc/apt/apt.conf.d/20apt-esm-hook.conf -f
    sudo systemctl mask apt-news.service
    sudo systemctl mask esm-cache.service
    sudo systemctl disable --now ubuntu-advantage
    sudo apt remove --purge ubuntu-advantage-tools
    sudo chmod -x /etc/update-motd.d/91-contract-ua-esm-status
    sudo chmod -x /etc/update-motd.d/50-motd-news
    sudo sed -Ezi.orig \
        -e 's/(def _output_esm_service_status.outstream, have_esm_service, service_type.:\n)/\1    return\n/' \
        -e 's/(def _output_esm_package_alert.*?\n.*?\n.:\n)/\1    return\n/' \
        /usr/lib/update-notifier/apt_check.py
    sudo /usr/lib/update-notifier/update-motd-updates-available --force
    ```

7. Disable unnecessary services:
   We don't need pipewire because the jetson is running headlessly which does not require audio capabilities, so we're just masking off the services.

    ```shell
    sudo systemctl --global disable --now pipewire-pulse.socket \
                                          pipewire-pulse.service \
                                          pipewire.socket \
                                          pipewire.service
    systemctl --user mask --now pipewire-pulse.socket \
                                pipewire-pulse.service \
                                pipewire.socket \
                                pipewire.service \
                                wireplumber.service
    systemctl --user daemon-reload
    ```

## Basic Setup & System Update

This is just some basic setup that is needed for the other steps.

1. Install apt-fast:

    ```shell
    sudo add-apt-repository ppa:apt-fast/stable
    sudo apt update
    sudo apt install apt-fast
    ```

    <!-- @formatter:off -->
    Select the following configuration options:

    - Package manager: `apt`
    - Maximum number of connections: `32`
    - Suppress apt-fast confirmation dialog: No

    TODO: section on configuring mirrors\
    TODO: section on configuring all the other useful parameters in apt-fast
    <!-- @formatter:on -->

2. Update packages:

    ```shell
    apt-fast update && apt-fast upgrade && apt-fast dist-upgrade
    apt-fast autoremove
    ```

3. Adjust fan speeds:
    1. Stop `nvfancontrol`:

       ```shell
       sudo systemctl stop nvfancontrol
       ```

4. Edit the `/etc/nvfancontrol.conf` file:
    - change the `cool` fan profile to:

      ```conf
      FAN_PROFILE cool {
          #TEMP   HYST    PWM     RPM
          0       0       255     6000
          10      0       255     6000
          11      0       215     5000
          40      0       215     4500
          75      0       170     3000
          105     0       40      1000
      }
      ```

    - `FAN_DEFAULT_PROFILE` to `cool`
5. Restart `nvfancontrol`:

    ```shell
    sudo rm /var/lib/nvfancontrol/status -rf
    sudo systemctl start nvfancontrol
    ```

## Fix NVIDIA Bullshit

NVIDIA has a lot of bs they've done here that needs to be fixed.

### Use Upstream OpenCV, FFmpeg, and GStreamer

NVIDIA ships a version of OpenCV, FFmpeg, and GStreamer that causes issues because it's different from the upstream Ubuntu version, so anything built against
the upstream Ubuntu version might just break.

```shell
sudo tee /etc/apt/preferences.d/00-l4t-fix-ffmpeg >/dev/null << EOF
# NVIDIA ships a version of ffmpeg different from
# the one present in the ubuntu repositories
# which causes packages depending on it to fail.

Package: ffmpeg
Pin: release o=Ubuntu
Pin-Priority: 1000
EOF
sudo tee /etc/apt/preferences.d/00-l4t-fix-opencv >/dev/null << EOF
# NVIDIA ships a version of opencv different from
# the one present in the ubuntu repositories
# which causes packages depending on it to fail.

Package: libopencv libopencv-* opencv-*
Pin: release o=Ubuntu
Pin-Priority: 1000
EOF
sudo tee /etc/apt/preferences.d/00-l4t-fix-gstreamer >/dev/null << EOF
# NVIDIA ships a version of gstreamer different from
# the one present in the ubuntu repositories
# which causes packages depending on it to fail.
#
# Note: it only felt necessary to add these few matches here
# if NVIDIA starts shipping more gstreamer packages, this may
# need to be adjusted.

Package: gstreamer* libgstreamer* libgstrtspserver*
Pin: release o=Ubuntu
Pin-Priority: 1000
EOF
apt-fast upgrade --allow-downgrades
```

Disable nvargus-daemon because we're not using argus for CSI/GMSL cameras it just uses a bunch of memory and we have absolutely zero use for it:

```shell
sudo systemctl disable --now nvargus-daemon.service
```

### Use tmpfs for `/tmp`

NVIDIA, in their infinite wisdom, decided that they do not need to use a tmpfs for `/tmp`. This is incredibly stupid. So, we will fix that.

```shell
sudo tee /etc/systemd/system/tmp.mount >/dev/null << EOF
[Unit]
Description=Temporary Directory /tmp
Documentation=https://systemd.io/TEMPORARY_DIRECTORIES
Documentation=man:file-hierarchy(7)
Documentation=https://systemd.io/API_FILE_SYSTEMS
Documentation=man:hier(7) man:tmpfs(5)
ConditionPathIsSymbolicLink=!/tmp
DefaultDependencies=no
Conflicts=umount.target
Before=local-fs.target umount.target
After=swap.target

[Mount]
What=tmpfs
Where=/tmp
Type=tmpfs
Options=mode=1777,strictatime,nosuid,nodev,size=50%%,nr_inodes=1m,x-systemd.graceful-option=usrquota
EOF
sudo rm /tmp/* /tmp/.* -rf
sudo systemctl daemon-reload
sudo systemctl enable --now tmp.mount
sudo reboot now
```

## Setup Swap

For some reason NVIDIA, in their infinite wisdom, does not create a swap file by default, so create one.

```shell
sudo dd if=/dev/zero of=/swapfile bs=1M count=8k status=progress oflag=direct
sudo chmod 0600 /swapfile
sudo mkswap -U clear /swapfile
sudo swapon /swapfile
echo -e '\n# swap file\n/swapfile            none                  swap           defaults                                     0 0' | sudo tee -a /etc/fstab
```

## Other \[TODO: Give This a Better name\]

1. Install basic utilities

    ```shell
    apt-fast install nano wget curl jq bash-completion ripgrep fd-find ncdu \
                     btop tree fzf screen tmux pv build-essential rsync zip \
                     bat nmap traceroute nethogs v4l-utils parallel imagemagick \
                     hyperfine hyfetch iperf3 iotop python3 pipx figlet lolcat
    ```

2. Install development dependencies

    <!-- @formatter:off -->
    ??? note "Compiler versions"

        As new versions of LLVM and GCC are released, the version used here should be updated.
    <!-- @formatter:on -->

    ```shell
    apt-fast install ccache valgrind ffmpeg gstreamer1.0-libav \
                     gstreamer1.0-opencv gstreamer1.0-pipewire \
                     gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
                     gstreamer1.0-tools gstreamer1.0-rtsp gstreamer1.0-vaapi \
                     build-essential mold ninja-build
    source /etc/os-release
    LLVM_VERSION=22
    sudo curl -fsSL 'https://apt.llvm.org/llvm-snapshot.gpg.key' -O '/etc/apt/trusted.gpg.d/apt.llvm.org.asc'
    sudo chmod a+r /etc/apt/trusted.gpg.d/apt.llvm.org.asc
    sudo tee /etc/apt/sources.list.d/llvm-repository.sources << EOF
    Types: deb deb-src
    Architectures: amd64 arm64
    Signed-By: /etc/apt/trusted.gpg.d/apt.llvm.org.asc
    URIs: https://apt.llvm.org/${VERSION_CODENAME}/
    Suites: llvm-toolchain-${VERSION_CODENAME}-${LLVM_VERSION}
    Components: main
    EOF
    GCC_VERSION=15
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    apt-fast update
    apt-fast install clang-$LLVM_VERSION lldb-$LLVM_VERSION lld-$LLVM_VERSION clangd-$LLVM_VERSION clang-tidy-$LLVM_VERSION clang-format-$LLVM_VERSION clang-tools-$LLVM_VERSION llvm-$LLVM_VERSION-tools llvm-$LLVM_VERSION gcc-$GCC_VERSION g++-$GCC_VERSION
    for version in 13 $GCC_VERSION; do
        sudo update-alternatives --install /usr/bin/gcc        gcc        /usr/bin/gcc-$version 50 \
                                 --slave   /usr/bin/g++        g++        /usr/bin/g++-$version \
                                 --slave   /usr/bin/gcc-ar     gcc-ar     /usr/bin/gcc-ar-$version \
                                 --slave   /usr/bin/gcc-nm     gcc-nm     /usr/bin/gcc-nm-$version \
                                 --slave   /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-$version
    done
    sudo update-alternatives --set gcc /usr/bin/gcc-$GCC_VERSION
    for version in $LLVM_VERSION; do
        sudo update-alternatives --install /usr/bin/clang       clang       /usr/bin/clang-$version 50 \
                                 --slave   /usr/bin/clang-cl    clang-cl    /usr/bin/clang-cl-$version \
                                 --slave   /usr/bin/clang-cpp   clang-cpp   /usr/bin/clang-cpp-$version \
                                 --slave   /usr/bin/clang++     clang++     /usr/bin/clang++-$version \
                                 --slave   /usr/bin/llvm-ar     llvm-ar     /usr/bin/llvm-ar-$version \
                                 --slave   /usr/bin/llvm-as     llvm-as     /usr/bin/llvm-as-$version \
                                 --slave   /usr/bin/llvm-nm     llvm-nm     /usr/bin/llvm-nm-$version \
                                 --slave   /usr/bin/llvm-ranlib llvm-ranlib /usr/bin/llvm-ranlib-$version
    done
    sudo update-alternatives --set clang /usr/bin/clang-$LLVM_VERSION
    ```

3. Install docker

    ```shell
    source /etc/os-release
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/trusted.gpg.d/docker.asc
    sudo chmod a+r /etc/apt/trusted.gpg.d/docker.asc
    sudo tee /etc/apt/sources.list.d/docker.sources << EOF
    Types: deb
    URIs: https://download.docker.com/linux/ubuntu
    Suites: ${VERSION_CODENAME}
    Components: stable
    Architectures: $(dpkg --print-architecture)
    Signed-By: /etc/apt/trusted.gpg.d/docker.asc
    EOF
    apt-fast update
    apt-fast install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    sudo systemctl disable --now containerd.service docker.socket docker.service
    ```

4. Configure ccache

    ```shell
    ccache -o max_size=16GB -o compression=true -o compression_level=5 -o sloppiness=pch_defines,time_macros
    ```

5. Set up oh-my-bash

    ```shell
    bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
    sed -Ei 's/OSH_THEME=".*"/OSH_THEME="powerline"/g'
    . ~/.bashrc
    ```

6. Set clockspeed to max (TODO: does this need to be done on every boot?)

    ```shell
    sudo jetson_clocks
    ```

## Install ROS

1. Install ROS 2

    ```shell
    source /etc/os-release
    apt-fast install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo add-apt-repository universe
    export ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')"
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    apt-fast update
    apt-fast install ros-dev-tools ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit
    apt-fast install python3-vcs2l
    sudo rosdep init
    rosdep update
    # TODO: add this to the bashrc: source /opt/ros/jazzy/setup.bash
    ```

2. Configure colcon mixins

    ```shell
    colcon mixin add default "https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml"
    colcon mixin update default
    ```

3. Switch to realtime kernel
    1. Modify `/etc/apt/sources.list.d/nvidia-l4t-apt-source.list` to add the `rt-kernel` repository, by adding this line

        ```
        deb https://repo.download.nvidia.com/jetson/rt-kernel <release> main
        ```

    2. Run:

        ```shell
        apt-fast update
        apt-fast install nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers nvidia-l4t-rt-kernel-oot-modules nvidia-l4t-display-rt-kernel nvidia-l4t-rt-kernel-nvgpu
        # TODO: add note that if we ever move from an orin nano to something else, the above line might not be correct
        ```

    3. Change the default kernel by modifying `/boot/extlinux/extlinux.conf` and setting `DEFAULT` to `real-time`, e.g.

        <!-- @formatter:off -->
        ```
        TIMEOUT 30
        DEFAULT real-time
        ```

        TODO: is this file generated? is it safe to just modify it like this?\
        TODO: this step might not even be necessary
        <!-- @formatter:on -->

    4. TODO: tuning?
    5. Reboot
4. Optimize the boot time
    1. Disable systemd-networkd wait online unit

       ```shell
       sudo systemctl disable --now systemd-networkd-wait-online.service
       sudo systemctl mask systemd-networkd-wait-online.service
       ```

## Setup CAN Bus

Install necessary dependencies

```shell
apt-fast install busybox can-utils nvidia-l4t-kernel-oot-modules
```

Install the files referenced in [CAN Bus documentation](../../hardware/canbus.md), then run

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=net --action=change
sudo systemctl daemon-reload
```

## Config

This section is a WIP

``` title="/etc/nanorc"
set atblanks
set autoindent
set brackets ""')>]}"
set indicator
set linenumbers
set matchbrackets "(<[{)>]}"
set multibuffer
set nonewlines
set punct "!.?;:"
set quotestr "^([ 	]*([!#%:;>|}]|//|/\*|\*/))+"
set regexp
set softwrap
set tabsize 4
set tabstospaces
set trimblanks
set whitespace ">·"
set wordbounds
set zap

extendsyntax python tabgives "    "
extendsyntax makefile tabgives "    "

## Highlight trailing whitespace
extendsyntax default color ,green "[[:space:]]+$"

bind ^H chopwordleft main
```

```shell title="~/.nanorc"
set titlecolor bold,lightwhite,blue
set promptcolor lightwhite,lightblack
set statuscolor bold,lightwhite,green
set errorcolor bold,lightwhite,red
set spotlightcolor black,lightyellow
set selectedcolor lightwhite,magenta
set stripecolor ,yellow
set scrollercolor cyan
set numbercolor cyan
set keycolor cyan
set functioncolor green
```

```shell title="/root/.nanorc"
set titlecolor bold,lightwhite,magenta
set promptcolor black,yellow
set statuscolor bold,lightwhite,magenta
set errorcolor bold,lightwhite,red
set spotlightcolor black,orange
set selectedcolor lightwhite,cyan
set stripecolor ,yellow
set scrollercolor magenta
set numbercolor magenta
set keycolor lightmagenta
set functioncolor magenta
```

```shell title="~/.inputrc"
### ctrl+arrows
# works in most terminals: xterm, gnome-terminal, terminator, st, sakura, termit, …
"\e[1;5C": forward-word
"\e[1;5D": backward-word
# urxvt
"\eOc": forward-word
"\eOd": backward-word

### ctrl+delete
"\e[3;5~": kill-word
# in this case, st misbehaves (even with tmux)
"\e[M": kill-word
# and of course, urxvt must be always special
"\e[3^": kill-word

### ctrl+backspace
"\C-h": backward-kill-word

### ctrl+shift+delete
"\e[3;6~": kill-line
# URxvt note: you have to disable Ctrl+Shift popup in ~/.Xresources:
# URxvt.iso14755: true
# URxvt.iso14755_52: false
"\e[3@": kill-line
# st sends same sequence as plain delete :(

# Color files by types
# Note that this may cause completion text blink in some terminals (e.g. xterm).
set colored-stats On
# Append char to indicate type
set visible-stats On
# Mark symlinked directories
set mark-symlinked-directories On
# Color the common prefix
set colored-completion-prefix On
# Color the common prefix in menu-complete
set menu-complete-display-prefix On
# case-insensitive completion
set completion-ignore-case On
```

```bash title="~/.oh-my-bash/custom/themes/powerline-purple/powerline-purple.theme.sh"
#!/usr/bin/env bash

source "$OSH/themes/powerline/powerline.base.sh"

PROMPT_CHAR=${POWERLINE_PROMPT_CHAR:=""}
POWERLINE_LEFT_SEPARATOR=${POWERLINE_LEFT_SEPARATOR:=""}

USER_INFO_SSH_CHAR=${POWERLINE_USER_INFO_SSH_CHAR:=" "}
USER_INFO_THEME_PROMPT_COLOR=54
USER_INFO_THEME_PROMPT_COLOR=92
USER_INFO_THEME_PROMPT_COLOR_SUDO=202

PYTHON_VENV_CHAR=${POWERLINE_PYTHON_VENV_CHAR:="❲p❳ "}
CONDA_PYTHON_VENV_CHAR=${POWERLINE_CONDA_PYTHON_VENV_CHAR:="❲c❳ "}
PYTHON_VENV_THEME_PROMPT_COLOR=35

SCM_NONE_CHAR=""
SCM_GIT_CHAR=${POWERLINE_SCM_GIT_CHAR:=" "}
SCM_THEME_PROMPT_CLEAN=""
SCM_THEME_PROMPT_DIRTY=""
SCM_THEME_PROMPT_CLEAN_COLOR=25
SCM_THEME_PROMPT_DIRTY_COLOR=160
SCM_THEME_PROMPT_STAGED_COLOR=30
SCM_THEME_PROMPT_UNSTAGED_COLOR=92
SCM_THEME_PROMPT_COLOR=${SCM_THEME_PROMPT_CLEAN_COLOR}

RVM_THEME_PROMPT_PREFIX=""
RVM_THEME_PROMPT_SUFFIX=""
RBENV_THEME_PROMPT_PREFIX=""
RBENV_THEME_PROMPT_SUFFIX=""
RUBY_THEME_PROMPT_COLOR=161
RUBY_CHAR=${POWERLINE_RUBY_CHAR:="❲r❳ "}

CWD_THEME_PROMPT_COLOR=240

LAST_STATUS_THEME_PROMPT_COLOR=52

CLOCK_THEME_PROMPT_COLOR=240

BATTERY_AC_CHAR=${BATTERY_AC_CHAR:="⚡"}
BATTERY_STATUS_THEME_PROMPT_GOOD_COLOR=70
BATTERY_STATUS_THEME_PROMPT_LOW_COLOR=208
BATTERY_STATUS_THEME_PROMPT_CRITICAL_COLOR=160

THEME_CLOCK_FORMAT=${THEME_CLOCK_FORMAT:="%H:%M:%S"}

IN_VIM_THEME_PROMPT_COLOR=245
IN_VIM_THEME_PROMPT_TEXT="vim"

POWERLINE_PROMPT=${POWERLINE_PROMPT:="clock user_info cwd scm python_venv"}

safe_append_prompt_command __powerline_prompt_command
```

`~/.bashrc`: TODO
`~/.profile`: TODO

## Clone Repository

```shell
gcl 'https://github.com/space-concordia-robotics/robot-repo-ros2.git'
cd robot-repo-ros2
vcs import --input external.repos external/
rosdep install -r -y --ignore-src --from-paths .
colcon build --packages-ignore foc2_gui
```

## Final Cleanup

```shell
apt-fast autoremove
apt-fast autoclean
apt-fast clean
```

## Upgrading to a New Release

See: [Software Packages and the Update Mechanism - NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/SoftwarePackagesAndTheUpdateMechanism.html#updating-to-a-new-minor-release)

TODO.

## Things To Do

This is a list of things that can be done in the future. Currently, none of these have been done, but if you decide to do any of these then you can give it a
try.

- [ ] custom oh my bash things:
    - [ ] ROS aliases
    - [ ] ROS completions
    - [ ] other stuff?
- [ ] automatically select best mirrors: https://github.com/vegardit/fast-apt-mirror.sh
    - [ ] Ubuntu mirrors
    - [ ] ROS mirrors
- [ ] alternate DDS
- [ ] install jetson-stats (Note: I, Will, previously decided against doing this due to there not being a ppa or ubuntu/debian repository that can be added)
- [ ] build opencv & ffmpeg from source with support for hardware acceleration?
    - [GitHub - jocover/jetson-ffmpeg: ffmpeg support on jetson nano · GitHub](https://github.com/jocover/jetson-ffmpeg)
    - [GitHub - Extend-Robotics/jetson-ffmpeg-keylost: ffmpeg support on jetson · GitHub](https://github.com/Extend-Robotics/jetson-ffmpeg-keylost)
