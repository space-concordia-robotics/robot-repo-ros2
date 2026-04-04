from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(mode="production", control="ros")
    sl.node(
        package="ublox_gps",
        executable="ublox_gps_node",
        name="gps_node",
        parameters=[sl.params(package="rover_description", file="ublox-gps.yaml")],
        remappings={
            "/gps_node/fix": "/ceres/gps/fix"
        }.items(),
        # namespace=""
    )

    return sl.launch_description()
