# Environment Setup

It is recommended that you use Ubuntu 24.04. Whether you use it in a VM, dual boot, or daily drive it, this is the best-supported approach.
If you choose to install ROS by some other method, then we may not be able to help you if you encounter any issues.

## ROS Installation

First, you need to install ROS by following the installation instructions found here:

```embed
url: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
```

Then, we need to install some necessary ROS packages:

```bash
apt install ros-dev-tools ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit python3-vcs2l
```

You then want to add the default colcon mixin repository:

```bash
colcon mixin add default "https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml"
colcon mixin update default
```

## Tooling & Dependencies

First, we're going to install some basic utilities:

```bash
apt-fast install nano wget curl jq bash-completion ripgrep fd-find ncdu \
                 btop tree fzf screen tmux pv build-essential rsync zip \
                 bat nmap traceroute nethogs v4l-utils parallel imagemagick \
                 hyperfine hyfetch iperf3 iotop python3 pipx figlet lolcat
```

Next, we need to install some development dependencies as we use a newer C++ version, for which there is no available compiler distributed with Ubuntu 24.04:

```shell
apt install ccache valgrind ffmpeg gstreamer1.0-libav \
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
apt update
apt install clang-$LLVM_VERSION lldb-$LLVM_VERSION lld-$LLVM_VERSION clangd-$LLVM_VERSION clang-tidy-$LLVM_VERSION clang-format-$LLVM_VERSION clang-tools-$LLVM_VERSION llvm-$LLVM_VERSION-tools llvm-$LLVM_VERSION gcc-$GCC_VERSION g++-$GCC_VERSION
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

Then, you want to set up ccache to speed up compilations:

```bash
ccache -o max_size=16GB -o compression=true -o compression_level=5 -o sloppiness=pch_defines,time_macros
```

## Clone Repository

Lastly, you're going to want to clone the repository:

```bash
git clone 'https://github.com/space-concordia-robotics/robot-repo-ros2.git'
cd robot-repo-ros2
vcs import --input external.repos external/
```

You can now install any additional packages that are required:

```bash
rosdep install -r -y --ignore-src --from-paths .
```

Now you should be able to compile all the packages in the repository (this may take some time the first time you run it):

```bash
colcon build
```

If you cannot compile everything for some reason, then please feel free to ask for help.
