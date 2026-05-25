import textwrap
from enum import Enum
from typing import Text, Dict, Optional, List

import launch
from launch import SomeSubstitutionsType, Substitution, LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

from .substitutions import Templated, OpaqueFunctionSubstitution
from .util import flatten_substitutions

__all__ = [
    "ros_gz_prefix",
    "GazeboType",
    "BridgeDirection",
    "ImageBridgeQoS",
    "GazeboBridge",
]

logger = launch.logging.get_logger(__name__)


class GazeboType(Enum):
    GZ = "gz"
    IGN = "ign"


def ros_gz_prefix() -> GazeboType:
    from os import environ

    for env in ("IGN_VERSION", "GZ_VERSION"):
        if env in environ:
            return GazeboType.IGN if environ[env] == "fortress" else GazeboType.GZ

    # guess from installed packages
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    for pkg in ("ros_gz", "ros_gz_sim"):
        try:
            get_package_share_directory(pkg)
            return GazeboType.GZ
        except PackageNotFoundError:
            continue
    return GazeboType.IGN


# ros <-> gz mapping
# from https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
# this is only used for development
# def generate_msg_map():
#     """
#     function to regenerate known messages from readme
#     """
#     import requests
#
#     response = requests.get("https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/ros2/ros_gz_bridge/README.md")
#
#     ros_gz_bridge_readme = response.text
#
#     out = ["    msg_map = {"]
#
#     for line in ros_gz_bridge_readme.splitlines():
#         if "|" in line and "/msg/" in line and "gz.msgs" in line:
#             _, ros, gz, _ = line.split("|")
#             out.append(f"    "{ros.strip()}": "{gz.strip()}",")
#     print("\n".join(out)[:-1] + "\n}")


ros_msg_map: Dict[Text, Text] = {
    "actuator_msgs/msg/Actuators": "gz.msgs.Actuators",
    "builtin_interfaces/msg/Time": "gz.msgs.Time",
    "geometry_msgs/msg/Point": "gz.msgs.Vector3d",
    "geometry_msgs/msg/Pose": "gz.msgs.Pose",
    "geometry_msgs/msg/PoseArray": "gz.msgs.Pose_V",
    "geometry_msgs/msg/PoseStamped": "gz.msgs.Pose",
    "geometry_msgs/msg/PoseWithCovariance": "gz.msgs.PoseWithCovariance",
    "geometry_msgs/msg/PoseWithCovarianceStamped": "gz.msgs.PoseWithCovariance",
    "geometry_msgs/msg/Quaternion": "gz.msgs.Quaternion",
    "geometry_msgs/msg/Transform": "gz.msgs.Pose",
    "geometry_msgs/msg/TransformStamped": "gz.msgs.Pose",
    "geometry_msgs/msg/Twist": "gz.msgs.Twist",
    "geometry_msgs/msg/TwistStamped": "gz.msgs.Twist",
    "geometry_msgs/msg/TwistWithCovariance": "gz.msgs.TwistWithCovariance",
    "geometry_msgs/msg/TwistWithCovarianceStamped": "gz.msgs.TwistWithCovariance",
    "geometry_msgs/msg/Vector3": "gz.msgs.Vector3d",
    "geometry_msgs/msg/Wrench": "gz.msgs.Wrench",
    "geometry_msgs/msg/WrenchStamped": "gz.msgs.Wrench",
    "gps_msgs/msg/GPSFix": "gz.msgs.NavSat",
    # "nav_msgs/msg/Odometry": "gz.msgs.Odometry",
    "nav_msgs/msg/Odometry": "gz.msgs.OdometryWithCovariance",
    "rcl_interfaces/msg/ParameterValue": "gz.msgs.Any",
    "ros_gz_interfaces/msg/Altimeter": "gz.msgs.Altimeter",
    "ros_gz_interfaces/msg/Contact": "gz.msgs.Contact",
    "ros_gz_interfaces/msg/Contacts": "gz.msgs.Contacts",
    "ros_gz_interfaces/msg/Dataframe": "gz.msgs.Dataframe",
    "ros_gz_interfaces/msg/Entity": "gz.msgs.Entity",
    "ros_gz_interfaces/msg/EntityWrench": "gz.msgs.EntityWrench",
    "ros_gz_interfaces/msg/Float32Array": "gz.msgs.Float_V",
    "ros_gz_interfaces/msg/GuiCamera": "gz.msgs.GUICamera",
    "ros_gz_interfaces/msg/JointWrench": "gz.msgs.JointWrench",
    "ros_gz_interfaces/msg/Light": "gz.msgs.Light",
    "ros_gz_interfaces/msg/LogicalCameraImage": "gz.msgs.LogicalCameraImage",
    # "ros_gz_interfaces/msg/ParamVec": "gz.msgs.Param",
    "ros_gz_interfaces/msg/ParamVec": "gz.msgs.Param_V",
    "ros_gz_interfaces/msg/SensorNoise": "gz.msgs.SensorNoise",
    "ros_gz_interfaces/msg/StringVec": "gz.msgs.StringMsg_V",
    "ros_gz_interfaces/msg/TrackVisual": "gz.msgs.TrackVisual",
    "ros_gz_interfaces/msg/VideoRecord": "gz.msgs.VideoRecord",
    "rosgraph_msgs/msg/Clock": "gz.msgs.Clock",
    "sensor_msgs/msg/BatteryState": "gz.msgs.BatteryState",
    "sensor_msgs/msg/CameraInfo": "gz.msgs.CameraInfo",
    "sensor_msgs/msg/FluidPressure": "gz.msgs.FluidPressure",
    "sensor_msgs/msg/Image": "gz.msgs.Image",
    "sensor_msgs/msg/Imu": "gz.msgs.IMU",
    "sensor_msgs/msg/JointState": "gz.msgs.Model",
    "sensor_msgs/msg/Joy": "gz.msgs.Joy",
    "sensor_msgs/msg/LaserScan": "gz.msgs.LaserScan",
    "sensor_msgs/msg/MagneticField": "gz.msgs.Magnetometer",
    "sensor_msgs/msg/NavSatFix": "gz.msgs.NavSat",
    "sensor_msgs/msg/PointCloud2": "gz.msgs.PointCloudPacked",
    "sensor_msgs/msg/Range": "gz.msgs.LaserScan",
    "std_msgs/msg/Bool": "gz.msgs.Boolean",
    "std_msgs/msg/ColorRGBA": "gz.msgs.Color",
    "std_msgs/msg/Empty": "gz.msgs.Empty",
    "std_msgs/msg/Float32": "gz.msgs.Float",
    "std_msgs/msg/Float64": "gz.msgs.Double",
    "std_msgs/msg/Header": "gz.msgs.Header",
    "std_msgs/msg/Int32": "gz.msgs.Int32",
    "std_msgs/msg/String": "gz.msgs.StringMsg",
    "std_msgs/msg/UInt32": "gz.msgs.UInt32",
    "tf2_msgs/msg/TFMessage": "gz.msgs.Pose_V",
    "trajectory_msgs/msg/JointTrajectory": "gz.msgs.JointTrajectory",
    "vision_msgs/msg/Detection2D": "gz.msgs.AnnotatedAxisAligned2DBox",
    "vision_msgs/msg/Detection2DArray": "gz.msgs.AnnotatedAxisAligned2DBox_V",
}

gz_msg_map: Dict[Text, Text] = {
    value: key for key, value in ros_msg_map.items()
}


class BridgeDirection(Enum):
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2
    BIDIRECTIONAL = 3


class ImageBridgeQoS(Enum):
    DEFAULT = 'default'
    SENSOR = 'sensor'
    SYSTEM_DEFAULT = 'system_default'


bridged_topic_template = """
- ${gz_exec_name}_topic_name: ${gz_topic}
  ${gz_exec_name}_type_name: ${gz_msg}
  ros_topic_name: ${ros_topic}
  ros_type_name: ${ros_msg}
  direction: ${direction}
"""

bridged_topic_template = textwrap.dedent(bridged_topic_template)


class GazeboBridge:
    _world_name: Optional[Text] = None
    _gz_exec: GazeboType = ros_gz_prefix()

    class Topic:
        __gz_topic: SomeSubstitutionsType
        __ros_topic: SomeSubstitutionsType
        __is_image: bool
        __direction: BridgeDirection
        __gz_msg: Text
        __ros_msg: Text

        # TODO 2026-03-09 (Will Free): Allow gz_topic & ros_topic to be SomeSubstitutionsType
        def __init__(
                self,
                gz_topic: SomeSubstitutionsType,
                ros_topic: SomeSubstitutionsType,
                direction: BridgeDirection,
                ros_msg: Optional[Text] = None,
                gz_msg: Optional[Text] = None,
        ):
            """
            Create a bridge instance to be passed to SimpleLauncher.create_gz_bridge
            """

            if gz_msg is None and ros_msg is None:
                logger.error(f"Provide either a ros or a gazebo message type for {gz_topic} bridged topic.")

            if ros_msg is None:
                if gz_msg not in gz_msg_map:
                    logger.error(f"Cannot build a ros <-> gz bridge for message \"{ros_msg}\": unknown type or give explicit gz_msg")
                    return
                ros_msg = gz_msg_map[gz_msg]
            elif gz_msg is None:
                if "/msg/" not in ros_msg:
                    ros_msg = ros_msg.replace("/", "/msg/")
                if ros_msg not in ros_msg_map:
                    logger.error(f"Cannot build a ros <-> gz bridge for message \"{ros_msg}\": unknown type or give explicit gz_msg")
                    return
                gz_msg = ros_msg_map[ros_msg]
            else:
                if "/msg/" not in ros_msg:
                    ros_msg = ros_msg.replace("/", "/msg/")

            self.__gz_msg = gz_msg
            self.__ros_msg = ros_msg

            if gz_msg is not None:
                self.__gz_msg = gz_msg
            elif ros_msg not in ros_msg_map:
                logger.error(f"Cannot build a ros <-> gz bridge for message \"{ros_msg}\": unknown type or give explicit gz_msg")
                return
            else:
                self.__gz_msg = ros_msg_map[ros_msg]

            self.__gz_topic = gz_topic
            self.__ros_topic = ros_topic

            # Images with gz2ros use ros_gz_image bridge
            self.__is_image = ros_msg == "sensor_msgs/msg/Image"
            self.__direction = direction

        @property
        def gz_topic(self) -> SomeSubstitutionsType:
            return self.__gz_topic

        @property
        def ros_topic(self) -> SomeSubstitutionsType:
            return self.__ros_topic

        @property
        def is_image(self) -> bool:
            return self.__is_image

        @property
        def direction(self) -> BridgeDirection:
            return self.__direction

        @property
        def ros_msg(self) -> Text:
            return self.__ros_msg

        @property
        def gz_msg(self) -> Text:
            if GazeboBridge._gz_exec == GazeboType.IGN:
                self.__gz_msg = self.__gz_msg.replace("gz.", "ignition.")
            return self.__gz_msg

        def yaml(self) -> Substitution:
            """
            use YAML-based config for other bridges
            - topic_name: "chatter"
              ign_topic_name: "ign_chatter"
              ros_type_name: "std_msgs/msg/String"
              ign_type_name: "ignition.msgs.StringMsg"
              direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
                                        # "IGN_TO_ROS" - Bridge Ignition topic to ROS
                                        # "ROS_TO_IGN" - Bridge ROS topic
            """

            match self.direction:
                case BridgeDirection.GZ_TO_ROS:
                    direction = f"{GazeboBridge._gz_exec.name}_TO_ROS"
                case BridgeDirection.ROS_TO_GZ:
                    direction = f"ROS_TO_{GazeboBridge._gz_exec.name}"
                case BridgeDirection.BIDIRECTIONAL:
                    direction = "BIDIRECTIONAL"

            return Templated(
                template=bridged_topic_template,
                args={
                    "gz_exec_name": GazeboBridge._gz_exec.value,
                    "gz_topic": self.gz_topic,
                    "ros_topic": self.ros_topic,
                    "gz_msg": self.gz_msg,
                    "ros_msg": self.ros_msg,
                    "direction": direction,
                },
            )

    topics: list[Topic]
    image_bridge_qos: Optional[ImageBridgeQoS]
    image_bridge_lazy: Optional[bool]
    image_bridge_subscription_heartbeat: Optional[int]

    def __init__(self):
        self.topics = []

    def image_bridge(self, qos: Optional[ImageBridgeQoS] = None, lazy: Optional[bool] = None, subscription_heartbeat: Optional[int] = None):
        self.image_bridge_qos = qos
        self.image_bridge_lazy = lazy
        self.image_bridge_subscription_heartbeat = subscription_heartbeat

    def add_topic(self, gz_topic, ros_topic, direction: BridgeDirection, ros_msg: Optional[Text] = None, gz_msg: Optional[Text] = None):
        topic = GazeboBridge.Topic(gz_topic, ros_topic, direction, ros_msg=ros_msg, gz_msg=gz_msg)
        self.topics.append(topic)

        if topic.is_image:
            # see: https://github.com/ros-perception/image_common/blob/4de91aef2daf41ddfd43f700417b35198b331980/image_transport/src/camera_common.cpp#L56-L70
            def remap_camera_info_topic(context: LaunchContext, *args, **kwargs) -> Text:
                image_topic: SomeSubstitutionsType = kwargs["image_topic"]

                image_topic: List[Substitution] = normalize_to_list_of_substitutions(image_topic)
                image_topic: Text = perform_substitutions(context, image_topic)

                paths = image_topic.split("/")[:-1]

                return "/".join(paths) + "/camera_info"

            gz_camera_info = OpaqueFunctionSubstitution(remap_camera_info_topic, kwargs={'image_topic': topic.gz_topic})
            ros_camera_info = OpaqueFunctionSubstitution(remap_camera_info_topic, kwargs={'image_topic': topic.ros_topic})

            cam_info_topic = GazeboBridge.Topic(gz_camera_info, ros_camera_info, BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/CameraInfo")
            self.topics.append(cam_info_topic)

    def add_clock(self):
        self.topics.append(GazeboBridge.clock())

    def add_joint_states(self, model: Optional[SomeSubstitutionsType] = None):
        self.topics.append(GazeboBridge.joint_states_topic(model))

    def add_tf(self):
        self.topics.append(GazeboBridge.tf_topic())

    @property
    def yaml(self) -> List[Substitution]:
        topics = self.topics

        return [topic.yaml() for topic in topics if not topic.is_image]

    @staticmethod
    def model_topic(model: SomeSubstitutionsType, topic: SomeSubstitutionsType) -> SomeSubstitutionsType:
        # TODO 2026-02-07 (Will Free): remove this method
        return flatten_substitutions(["/model/", model, "/", topic])

    @staticmethod
    def clock() -> Topic:
        """
        Classical GZ -> ROS bridge for the clock topic
        """
        return GazeboBridge.Topic("/clock", "/clock", BridgeDirection.GZ_TO_ROS, ros_msg="rosgraph_msgs/Clock")

    @staticmethod
    def joint_states_topic(model: Optional[SomeSubstitutionsType] = None) -> Topic:
        """
        Classical GZ -> ROS bridge for the joint states of a given model
        """
        if model is None:
            joint_states_gz_topic = "/joint_states"
        else:
            joint_states_gz_topic = GazeboBridge.model_topic(model, "joint_state")
        return GazeboBridge.Topic(joint_states_gz_topic, "joint_states", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/JointState")

    @staticmethod
    def tf_topic() -> Topic:
        """
        Classical GZ -> ROS bridge for the tf topic
        """
        return GazeboBridge.Topic("/tf", "/tf", BridgeDirection.GZ_TO_ROS, ros_msg="tf2_msgs/TFMessage")
