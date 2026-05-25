"""ROS 2 node: DepthAI OAK stereo + spatial YOLO, plus CameraInfo and vision_msgs detections.

Shared lookups and CameraInfo/helpers live in :mod:`luxonis_ros.dai_ros_utils`.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    Pose2D,
)

try:
    import depthai as dai
except ImportError:
    dai = None  # type: ignore[misc, assignment]

from luxonis_ros.dai_ros_utils import (
    NN_BASENAME,
    build_camera_info,
    default_nn_archive_path,
    hypothesis_with_pose,
    identity_orientation,
    norm_xy_to_px,
)


# ---------------------------------------------------------------------------
# vision_msgs from SpatialImgDetections
# ---------------------------------------------------------------------------
def _spatial_detections_to_ros(
    src: Any,
    header: Header,
    image_width: int,
    image_height: int,
    *,
    mm_to_m: float,
) -> tuple[Detection3DArray, Detection2DArray]:
    dets3 = Detection3DArray(header=header)
    dets2 = Detection2DArray(header=header)
    w, h = image_width, image_height

    for d in src.detections:
        sc = d.spatialCoordinates
        mx = float(sc.x) * mm_to_m
        my = float(sc.y) * mm_to_m
        mz = float(sc.z) * mm_to_m
        label = d.labelName or str(d.label)
        ohp3 = hypothesis_with_pose(label, d.confidence, mx, my, mz)

        det3 = Detection3D()
        det3.header = header
        det3.results = [ohp3]
        det3.id = label
        bbox3 = BoundingBox3D()
        bbox3.center.position.x = mx
        bbox3.center.position.y = my
        bbox3.center.position.z = mz
        bbox3.center.orientation = identity_orientation()
        bbox3.size = Vector3(x=0.0, y=0.0, z=0.0)
        det3.bbox = bbox3
        dets3.detections.append(det3)

        xmin = norm_xy_to_px(d.xmin, w)
        xmax = norm_xy_to_px(d.xmax, w)
        ymin = norm_xy_to_px(d.ymin, h)
        ymax = norm_xy_to_px(d.ymax, h)
        cx = (xmin + xmax) * 0.5
        cy = (ymin + ymax) * 0.5
        sx = max(xmax - xmin, 0.0)
        sy = max(ymax - ymin, 0.0)
        bb2 = BoundingBox2D()
        bb2.center = Pose2D()
        bb2.center.position.x = cx
        bb2.center.position.y = cy
        bb2.center.theta = 0.0
        bb2.size_x = sx
        bb2.size_y = sy
        ohp2 = hypothesis_with_pose(label, d.confidence, mx, my, mz)
        det2 = Detection2D()
        det2.header = header
        det2.results = [ohp2]
        det2.id = label
        det2.bbox = bb2
        dets2.detections.append(det2)

    return dets3, dets2


# ---------------------------------------------------------------------------
# DepthAI worker (same graph as main.py ``oakd_yolo`` preview path, minus encoders)
# ---------------------------------------------------------------------------
@dataclass
class OakSpatialSharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    rgb_frame: Any = None
    depth_frame: Any = None
    spatial_detections: Any = None
    calib: Any = None
    error: Optional[str] = None


class OakSpatialWorker(threading.Thread):
    def __init__(
        self,
        *,
        device_mxid: str,
        nn_archive_path: str,
        fps: float,
        confidence_threshold: float,
        depth_lower_threshold: int,
        depth_upper_threshold: int,
        align_depth_to_rgb: bool,
        state: OakSpatialSharedState,
        stop_event: threading.Event,
        log_fn: Any,
    ) -> None:
        super().__init__(daemon=True)
        self._device_mxid = device_mxid.strip()
        self._nn_archive_path = nn_archive_path
        self._fps = fps
        self._confidence_threshold = confidence_threshold
        self._depth_lower_threshold = depth_lower_threshold
        self._depth_upper_threshold = depth_upper_threshold
        self._align_depth_to_rgb = align_depth_to_rgb
        self._state = state
        self._stop_event = stop_event
        self._log = log_fn

    def run(self) -> None:
        if dai is None:
            self._state.error = 'Python package depthai is not installed.'
            self._log.error(self._state.error)
            return
        try:
            self._run_pipeline()
        except Exception as e:  # noqa: BLE001
            self._state.error = str(e)
            self._log.error(f'DepthAI pipeline failed: {e}')

    def _resolve_device_info(self) -> Any:
        if self._device_mxid:
            return dai.DeviceInfo(self._device_mxid)
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            raise RuntimeError('No DepthAI devices found (connect USB / check permissions).')
        return devices[0]

    def _run_pipeline(self) -> None:
        device_info = self._resolve_device_info()
        device = dai.Device(device_info)
        try:
            device.setIrLaserDotProjectorIntensity(1.0)
            device.setIrFloodLightIntensity(0.5)
        except Exception:
            pass
        calib = device.readCalibration()
        with self._state.lock:
            self._state.calib = calib

        with dai.Pipeline(device) as pipeline:
            rgb_q, det_q, depth_q = self._build_spatial_pipeline(pipeline)
            pipeline.start()
            while pipeline.isRunning() and not self._stop_event.is_set():
                with self._state.lock:
                    if rgb_q.has():
                        self._state.rgb_frame = rgb_q.get()
                    if det_q.has():
                        self._state.spatial_detections = det_q.get()
                    if depth_q.has():
                        self._state.depth_frame = depth_q.get()
                time.sleep(0.002)

    def _build_spatial_pipeline(self, pipeline: Any) -> tuple[Any, Any, Any]:
        cam_rgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A,
            sensorFps=self._fps,
        )
        mono_left = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B,
            sensorFps=self._fps,
        )
        mono_right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C,
            sensorFps=self._fps,
        )
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setRectification(True)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        # Align depth to the RGB camera so the published depth image shares
        # frame_id, timestamp basis, and intrinsics with image_raw.
        if self._align_depth_to_rgb:
            try:
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            except Exception as e:  # noqa: BLE001
                self._log.warn(
                    f'setDepthAlign(CAM_A) failed ({e}); depth will be in rectified-right frame.'
                )
        mono_left.requestOutput((640, 400)).link(stereo.left)
        mono_right.requestOutput((640, 400)).link(stereo.right)
        archive = dai.NNArchive(self._nn_archive_path)
        spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
            cam_rgb,
            stereo,
            archive,
        )
        spatial_nn.input.setBlocking(False)
        spatial_nn.setConfidenceThreshold(self._confidence_threshold)
        spatial_nn.setDepthLowerThreshold(self._depth_lower_threshold)
        spatial_nn.setDepthUpperThreshold(self._depth_upper_threshold)
        rgb_q = spatial_nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
        det_q = spatial_nn.out.createOutputQueue(maxSize=4, blocking=False)
        # Use passthroughDepth so the depth frame the NN actually consumed
        # is the one we publish (one-to-one with each detection batch).
        depth_q = spatial_nn.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
        return rgb_q, det_q, depth_q


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class SpatialCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('spatial_camera')

        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_name', 'oak')
        self.declare_parameter('use_depthai', False)
        self.declare_parameter('device_mxid', '')
        self.declare_parameter('nn_archive_path', default_nn_archive_path())
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_lower_threshold', 200)
        self.declare_parameter('depth_upper_threshold', 8000)
        self.declare_parameter('spatial_xyz_mm_to_m', 0.001)
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('depth_image_topic', 'depth/image_raw')
        self.declare_parameter('depth_camera_info_topic', 'depth/camera_info')
        self.declare_parameter('detections_3d_topic', 'detections_3d')
        self.declare_parameter('detections_2d_topic', 'detections_2d')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('align_depth_to_rgb', True)
        self.declare_parameter('use_placeholder_image', True)
        self.declare_parameter('placeholder_width', 640)
        self.declare_parameter('placeholder_height', 480)

        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._use_depthai = self.get_parameter('use_depthai').get_parameter_value().bool_value
        self._publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._mm_to_m = self.get_parameter('spatial_xyz_mm_to_m').get_parameter_value().double_value

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        depth_cam_info_topic = (
            self.get_parameter('depth_camera_info_topic').get_parameter_value().string_value
        )
        det3_topic = self.get_parameter('detections_3d_topic').get_parameter_value().string_value
        det2_topic = self.get_parameter('detections_2d_topic').get_parameter_value().string_value

        self._publish_depth = self.get_parameter('publish_depth').get_parameter_value().bool_value
        self._align_depth_to_rgb = (
            self.get_parameter('align_depth_to_rgb').get_parameter_value().bool_value
        )

        self._bridge = CvBridge()
        self._camera_info_msg: CameraInfo | None = None
        self._camera_info_sent_dims: tuple[int, int] | None = None
        self._depth_camera_info_msg: CameraInfo | None = None
        self._depth_camera_info_sent_dims: tuple[int, int] | None = None

        self._image_pub = self.create_publisher(Image, image_topic, qos_profile_sensor_data)

        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._cam_info_pub = self.create_publisher(CameraInfo, camera_info_topic, cam_qos)
        self._depth_image_pub = self.create_publisher(
            Image, depth_image_topic, qos_profile_sensor_data
        )
        self._depth_cam_info_pub = self.create_publisher(CameraInfo, depth_cam_info_topic, cam_qos)
        self._det3_pub = self.create_publisher(Detection3DArray, det3_topic, 10)
        self._det2_pub = self.create_publisher(Detection2DArray, det2_topic, 10)

        period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.033
        self._timer = self.create_timer(period, self._on_publish_tick)

        self._oak_state: OakSpatialSharedState | None = None
        self._oak_stop: threading.Event | None = None
        self._oak_thread: OakSpatialWorker | None = None

        self._use_placeholder_image = (
            self.get_parameter('use_placeholder_image').get_parameter_value().bool_value
        )
        self._placeholder_w = (
            self.get_parameter('placeholder_width').get_parameter_value().integer_value
        )
        self._placeholder_h = (
            self.get_parameter('placeholder_height').get_parameter_value().integer_value
        )
        self._ph_seq = 0

        if self._use_depthai:
            nn_path = self.get_parameter('nn_archive_path').get_parameter_value().string_value.strip()
            if not nn_path:
                nn_path = default_nn_archive_path()
            if not nn_path or not Path(nn_path).is_file():
                self.get_logger().error(
                    f'NN archive not found: {(nn_path or "empty")!r}. '
                    'Set nn_archive_path or LUXONIS_NN_ARCHIVE, or install the model under '
                    f'share/luxonis_ros/models/{_NN_BASENAME}, or keep it in luxonis_scripts/ in the repo.'
                )
            self._oak_stop = threading.Event()
            self._oak_state = OakSpatialSharedState()
            self._oak_thread = OakSpatialWorker(
                device_mxid=self.get_parameter('device_mxid').get_parameter_value().string_value,
                nn_archive_path=nn_path,
                fps=float(self.get_parameter('fps').get_parameter_value().double_value),
                confidence_threshold=float(
                    self.get_parameter('confidence_threshold').get_parameter_value().double_value
                ),
                depth_lower_threshold=int(
                    self.get_parameter('depth_lower_threshold').get_parameter_value().integer_value
                ),
                depth_upper_threshold=int(
                    self.get_parameter('depth_upper_threshold').get_parameter_value().integer_value
                ),
                align_depth_to_rgb=self._align_depth_to_rgb,
                state=self._oak_state,
                stop_event=self._oak_stop,
                log_fn=self.get_logger(),
            )
            self._oak_thread.start()

        cam_name = self.get_parameter('camera_name').get_parameter_value().string_value
        mode = 'depthai_oak_spatial' if self._use_depthai else 'placeholder'
        self.get_logger().info(
            f'Spatial camera node started mode={mode!r} camera_name={cam_name!r} '
            f'frame_id={self._frame_id!r} image_topic={image_topic!r}'
        )

    def shutdown_resources(self) -> None:
        if self._oak_stop is not None:
            self._oak_stop.set()
        if self._oak_thread is not None:
            self._oak_thread.join(timeout=8.0)

    def _bgr_placeholder(self) -> np.ndarray:
        h, w = self._placeholder_h, self._placeholder_w
        img = np.zeros((h, w, 3), dtype=np.uint8)
        square = 40
        phase = self._ph_seq % 2
        for i in range(0, h, square):
            for j in range(0, w, square):
                light = (((i // square) + (j // square) + phase) % 2) == 0
                color = (200, 200, 200) if light else (45, 45, 45)
                i2 = min(i + square, h)
                j2 = min(j + square, w)
                img[i:i2, j:j2] = color
        return img

    def _ensure_camera_info(self, width: int, height: int) -> None:
        if self._camera_info_sent_dims == (width, height) and self._camera_info_msg is not None:
            return
        if self._oak_state is None or self._oak_state.calib is None:
            return
        if dai is None:
            return
        try:
            self._camera_info_msg = build_camera_info(
                self._oak_state.calib,
                dai.CameraBoardSocket.CAM_A,
                width,
                height,
                self._frame_id,
            )
            self._camera_info_sent_dims = (width, height)
            self.get_logger().info(f'CameraInfo built for {width}x{height}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Failed to build CameraInfo: {e}')

    def _ensure_depth_camera_info(self, width: int, height: int) -> None:
        if (
            self._depth_camera_info_sent_dims == (width, height)
            and self._depth_camera_info_msg is not None
        ):
            return
        if self._oak_state is None or self._oak_state.calib is None or dai is None:
            return
        # When depth is aligned to RGB it uses CAM_A intrinsics, otherwise
        # the StereoDepth default frame is the rectified right (CAM_C).
        socket = (
            dai.CameraBoardSocket.CAM_A
            if self._align_depth_to_rgb
            else dai.CameraBoardSocket.CAM_C
        )
        try:
            self._depth_camera_info_msg = build_camera_info(
                self._oak_state.calib,
                socket,
                width,
                height,
                self._frame_id,
            )
            self._depth_camera_info_sent_dims = (width, height)
            self.get_logger().info(
                f'Depth CameraInfo built for {width}x{height} '
                f'(socket={"CAM_A" if self._align_depth_to_rgb else "CAM_C"})'
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Failed to build depth CameraInfo: {e}')

    def _on_publish_tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._frame_id)

        if self._use_depthai:
            self._publish_depthai(header)
            return

        if self._use_placeholder_image:
            bgr = self._bgr_placeholder()
        else:
            self.get_logger().warn(
                'use_placeholder_image=false and use_depthai=false; publishing gray.',
                throttle_duration_sec=5.0,
            )
            bgr = np.full((self._placeholder_h, self._placeholder_w, 3), 96, dtype=np.uint8)

        msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)
        self._ph_seq += 1

    def _publish_depthai(self, header: Header) -> None:
        assert self._oak_state is not None

        if self._oak_state.error:
            self.get_logger().error(
                f'DepthAI error: {self._oak_state.error}',
                throttle_duration_sec=5.0,
            )

        with self._oak_state.lock:
            rgb_frame = self._oak_state.rgb_frame
            depth_frame = self._oak_state.depth_frame
            spatial_dets = self._oak_state.spatial_detections

        if rgb_frame is None:
            return

        try:
            bgr = rgb_frame.getCvFrame()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'getCvFrame failed: {e}', throttle_duration_sec=2.0)
            return

        h, w = bgr.shape[:2]
        self._ensure_camera_info(w, h)

        img_msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        img_msg.header = header
        self._image_pub.publish(img_msg)

        if self._camera_info_msg is not None:
            ci = self._camera_info_msg
            ci.header.stamp = header.stamp
            ci.header.frame_id = self._frame_id
            self._cam_info_pub.publish(ci)

        if self._publish_depth and depth_frame is not None:
            self._publish_depth_frame(depth_frame, header)

        if spatial_dets is not None:
            det3, det2 = _spatial_detections_to_ros(
                spatial_dets,
                header,
                w,
                h,
                mm_to_m=self._mm_to_m,
            )
            self._det3_pub.publish(det3)
            self._det2_pub.publish(det2)
            self._log_detections(det3)

    def _publish_depth_frame(self, depth_frame: Any, header: Header) -> None:
        try:
            depth = depth_frame.getCvFrame()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'depth getCvFrame failed: {e}', throttle_duration_sec=2.0
            )
            return
        if depth is None or depth.size == 0:
            return
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        dh, dw = depth.shape[:2]
        self._ensure_depth_camera_info(dw, dh)

        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header = header
        self._depth_image_pub.publish(depth_msg)

        if self._depth_camera_info_msg is not None:
            dci = self._depth_camera_info_msg
            dci.header.stamp = header.stamp
            dci.header.frame_id = self._frame_id
            self._depth_cam_info_pub.publish(dci)

    def _log_detections(self, det3: Detection3DArray) -> None:
        """Throttled human-readable log of detected class + xyz (meters)."""
        if not det3.detections:
            return
        parts = [
            f'{d.id} xyz=({d.bbox.center.position.x:+.3f}, '
            f'{d.bbox.center.position.y:+.3f}, '
            f'{d.bbox.center.position.z:+.3f}) m'
            for d in det3.detections
        ]
        self.get_logger().info('; '.join(parts), throttle_duration_sec=1.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SpatialCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_resources()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()