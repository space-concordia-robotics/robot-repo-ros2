export CMAKE_PREFIX_PATH=/home/solonovamax/Programming/C++/robotics-prototype/.pixi/envs/default

function cmake() {
    /home/solonovamax/Programming/C++/robotics-prototype/.pixi/envs/default/bin/cmake "$0"
}

cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/solonovamax/Programming/C++/robotics-prototype/robot/rospackages/install/
cmake --build /home/solonovamax/Programming/C++/robotics-prototype/robot/rospackages/src/build/rtsp_camera -- -j8 -l8
cmake --build /home/solonovamax/Programming/C++/robotics-prototype/robot/rospackages/src/build/rtsp_camera -- -j8 -l8