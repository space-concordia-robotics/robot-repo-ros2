"""ROS 2 node: OAK-FFC-4P — four RGB streams with on-device YOLO (DetectionNetwork, no depth)."""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
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
from vision_msgs.msg import Detection2DArray

from luxonis_ros.dai_ros_utils import (
    NN_BASENAME,
    build_camera_info_with_fallback,
    default_nn_archive_path,
    img_detections_to_detection2d_array,
)

try:
    import depthai as dai
except ImportError:
    dai = None  # type: ignore[misc, assignment]


def _ffc_socket_slug_pairs() -> list[tuple[Any, str]]:
    if dai is None:
        return []
    return [
        (dai.CameraBoardSocket.CAM_A, 'front'),
        (dai.CameraBoardSocket.CAM_B, 'right'),
        (dai.CameraBoardSocket.CAM_C, 'left'),
        (dai.CameraBoardSocket.CAM_D, 'back'),
    ]


def _filter_ffc_layout(active_cameras_param: str) -> list[tuple[Any, str]]:
    """Return layout subset; ``all`` or empty string means all four."""
    full = _ffc_socket_slug_pairs()
    raw = active_cameras_param.strip().lower()
    if not raw or raw == 'all':
        return full
    want = {x.strip().lower() for x in active_cameras_param.split(',') if x.strip()}
    allowed = {sl for _, sl in full}
    unknown = want - allowed
    if unknown:
        raise ValueError(
            f'active_cameras contains unknown slug(s): {sorted(unknown)}. '
            f'Use comma-separated subset of {sorted(allowed)}.'
        )
    out = [(sock, slug) for sock, slug in full if slug in want]
    if not out:
        raise ValueError('active_cameras produced an empty layout.')
    return out


def apply_depthai_poe_watchdog_defaults(
    *,
    watchdog_ms: int,
    watchdog_initial_delay_ms: int,
    bootup_timeout_ms: int,
) -> None:
    """Ethernet/PoE: relax host↔device keepalive timeouts (respect pre-set env vars)."""
    if watchdog_ms > 0:
        os.environ.setdefault('DEPTHAI_WATCHDOG', str(watchdog_ms))
    if watchdog_initial_delay_ms > 0:
        os.environ.setdefault('DEPTHAI_WATCHDOG_INITIAL_DELAY', str(watchdog_initial_delay_ms))
    if bootup_timeout_ms > 0:
        os.environ.setdefault('DEPTHAI_BOOTUP_TIMEOUT', str(bootup_timeout_ms))


@dataclass
class FfcStreamBuffers:
    rgb_frame: Any = None
    img_detections: Any = None


@dataclass
class FfcSharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    calib: Any = None
    streams: dict[str, FfcStreamBuffers] = field(default_factory=dict)
    error: Optional[str] = None


_OUTPUT_QUEUE_DEPTH = 16


class FfcQuadWorker(threading.Thread):
    def __init__(
        self,
        *,
        layout: list[tuple[Any, str]],
        device_mxid: str,
        nn_archive_path: str,
        fps: float,
        confidence_threshold: float,
        state: FfcSharedState,
        stop_event: threading.Event,
        log_fn: Any,
    ) -> None:
        super().__init__(daemon=True)
        self._layout = layout
        self._device_mxid = device_mxid.strip()
        self._nn_archive_path = nn_archive_path
        self._fps = fps
        self._confidence_threshold = confidence_threshold
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
            self._log.error(f'DepthAI FFC pipeline failed: {e}')

    def _resolve_device_info(self) -> Any:
        if self._device_mxid:
            return dai.DeviceInfo(self._device_mxid)
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            raise RuntimeError(
                'No DepthAI devices found. For PoE/Ethernet, pass the camera IP as '
                'device_mxid (launch: device_mxid:=10.x.x.x). In Docker, ensure the '
                'container can route to that host (--network host or VPC routes). '
                'USB devices need /dev/bus/usb in the container.'
            )
        return devices[0]

    def _run_pipeline(self) -> None:
        device_info = self._resolve_device_info()
        target = self._device_mxid or '(first discovered device)'
        self._log.info(f'DepthAI opening device {target!r} ({len(self._layout)} camera branch(es))...')
        try:
            device = dai.Device(device_info)
        except Exception as e:
            hint = ''
            if self._device_mxid:
                hint = (
                    ' PoE/LAN often fails from cloud/container subnets with no route to the camera '
                    '(e.g. host on 172.31.x vs camera on 10.240.x needs VPC peering or run on LAN). '
                    'Try TCP reachability from this host or Docker `--network host` on a machine '
                    'that can reach the camera; see DepthAI Ethernet / XLink troubleshooting.'
                )
            raise RuntimeError(f'Failed to open DepthAI device {target!r}: {e!s}.{hint}') from e

        layout = list(self._layout)
        calib = device.readCalibration()
        with self._state.lock:
            self._state.calib = calib
            self._state.streams = {slug: FfcStreamBuffers() for _, slug in layout}

        with dai.Pipeline(device) as pipeline:
            archive = dai.NNArchive(self._nn_archive_path)
            queues: list[tuple[str, Any, Any, Any]] = []
            for cam_socket, slug in layout:
                cam = pipeline.create(dai.node.Camera).build(cam_socket, sensorFps=self._fps)
                det = pipeline.create(dai.node.DetectionNetwork).build(cam, archive)
                det.input.setBlocking(False)
                det.setConfidenceThreshold(self._confidence_threshold)
                rgb_q = det.passthrough.createOutputQueue(
                    maxSize=_OUTPUT_QUEUE_DEPTH, blocking=False
                )
                det_q = det.out.createOutputQueue(maxSize=_OUTPUT_QUEUE_DEPTH, blocking=False)
                queues.append((slug, cam_socket, rgb_q, det_q))

            pipeline.start()
            while pipeline.isRunning() and not self._stop_event.is_set():
                try:
                    with self._state.lock:
                        for slug, _cam_socket, rgb_q, det_q in queues:
                            buf = self._state.streams[slug]
                            if rgb_q.has():
                                buf.rgb_frame = rgb_q.get()
                            if det_q.has():
                                buf.img_detections = det_q.get()
                except Exception as e:  # noqa: BLE001
                    msg = str(e)
                    self._state.error = msg
                    self._log.error(
                        f'XLink / queue read failed (link overload or watchdog): {msg}. '
                        'Try lower fps, fewer cameras (active_cameras), or increase '
                        'depthai_watchdog_* parameters / DEPTHAI_WATCHDOG env.'
                    )
                    break
                time.sleep(0.001)


class FfcCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('ffc_quad_camera')

        self.declare_parameter('device_mxid', '')
        self.declare_parameter('nn_archive_path', default_nn_archive_path())
        self.declare_parameter('fps', 4.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('topic_namespace', 'ffc')
        self.declare_parameter('frame_prefix', 'ffc')
        self.declare_parameter(
            'active_cameras',
            'all',
            descriptor=ParameterDescriptor(
                description='Comma-separated slugs: front,right,left,back or "all". '
                'Use a subset to cut PoE/XLink load (e.g. front on slow links).',
            ),
        )
        self.declare_parameter(
            'depthai_watchdog_ms',
            4500,
            descriptor=ParameterDescriptor(
                description='PoE keepalive (ms). Set before device open; 0 leaves env unchanged.',
            ),
        )
        self.declare_parameter(
            'depthai_watchdog_initial_delay_ms',
            60000,
            descriptor=ParameterDescriptor(
                description='Initial watchdog delay over Ethernet (ms); 0 leaves env unchanged.',
            ),
        )
        self.declare_parameter(
            'depthai_bootup_timeout_ms',
            60000,
            descriptor=ParameterDescriptor(
                description='Device boot/connect timeout (ms); 0 leaves env unchanged.',
            ),
        )

        self._publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._topic_ns = self.get_parameter('topic_namespace').get_parameter_value().string_value.strip(
            '/'
        )
        self._frame_prefix = self.get_parameter('frame_prefix').get_parameter_value().string_value.strip(
            '_'
        )

        active_raw = self.get_parameter('active_cameras').get_parameter_value().string_value
        try:
            self._layout = _filter_ffc_layout(active_raw)
        except ValueError as e:
            self.get_logger().error(str(e))
            self._layout = []

        apply_depthai_poe_watchdog_defaults(
            watchdog_ms=int(self.get_parameter('depthai_watchdog_ms').get_parameter_value().integer_value),
            watchdog_initial_delay_ms=int(
                self.get_parameter('depthai_watchdog_initial_delay_ms').get_parameter_value().integer_value
            ),
            bootup_timeout_ms=int(
                self.get_parameter('depthai_bootup_timeout_ms').get_parameter_value().integer_value
            ),
        )

        self._bridge = CvBridge()

        self._pub_image: dict[str, Any] = {}
        self._pub_cam_info: dict[str, Any] = {}
        self._pub_det2: dict[str, Any] = {}
        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        for _sock, slug in self._layout:
            base = f'{self._topic_ns}/{slug}'
            self._pub_image[slug] = self.create_publisher(Image, f'{base}/image_raw', qos_profile_sensor_data)
            self._pub_cam_info[slug] = self.create_publisher(CameraInfo, f'{base}/camera_info', cam_qos)
            self._pub_det2[slug] = self.create_publisher(
                Detection2DArray,
                f'{base}/detections_2d',
                10,
            )

        self._camera_info_cache: dict[str, CameraInfo] = {}
        self._camera_info_dims: dict[str, tuple[int, int]] = {}
        self._camera_info_fallback_logged: set[str] = set()

        period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.033
        self._timer = self.create_timer(period, self._on_publish_tick)

        self._ffc_state: FfcSharedState | None = None
        self._ffc_stop: threading.Event | None = None
        self._ffc_thread: FfcQuadWorker | None = None

        if dai is None:
            self.get_logger().error('depthai is not installed; FFC node will not start a pipeline.')
        elif not self._layout:
            self.get_logger().error('No valid camera layout; fix active_cameras parameter.')
        else:
            nn_path = self.get_parameter('nn_archive_path').get_parameter_value().string_value.strip()
            if not nn_path:
                nn_path = default_nn_archive_path()
            if not nn_path or not Path(nn_path).is_file():
                self.get_logger().error(
                    f'NN archive not found: {(nn_path or "empty")!r}. '
                    'Set nn_archive_path or LUXONIS_NN_ARCHIVE, or install the model under '
                    f'share/luxonis_ros/models/{NN_BASENAME}, or keep it in luxonis_scripts/.'
                )
            else:
                self._ffc_stop = threading.Event()
                self._ffc_state = FfcSharedState()
                self._ffc_thread = FfcQuadWorker(
                    layout=list(self._layout),
                    device_mxid=self.get_parameter('device_mxid').get_parameter_value().string_value,
                    nn_archive_path=nn_path,
                    fps=float(self.get_parameter('fps').get_parameter_value().double_value),
                    confidence_threshold=float(
                        self.get_parameter('confidence_threshold').get_parameter_value().double_value
                    ),
                    state=self._ffc_state,
                    stop_event=self._ffc_stop,
                    log_fn=self.get_logger(),
                )
                self._ffc_thread.start()

        _dev_id = self.get_parameter('device_mxid').get_parameter_value().string_value.strip()
        slugs = [s for _, s in self._layout]
        self.get_logger().info(
            f'FFC quad camera node started ns={self._topic_ns!r} '
            f'frame_prefix={self._frame_prefix!r} publish_hz={self._publish_rate} '
            f'device_mxid={_dev_id!r} active_cameras={slugs!r} '
            f'(DEPTHAI_WATCHDOG={os.environ.get("DEPTHAI_WATCHDOG")!s})'
        )

    def shutdown_resources(self) -> None:
        if self._ffc_stop is not None:
            self._ffc_stop.set()
        if self._ffc_thread is not None:
            self._ffc_thread.join(timeout=30.0)

    def _frame_id_for_slug(self, slug: str) -> str:
        return f'{self._frame_prefix}_{slug}_optical_frame'

    def _ensure_camera_info(self, slug: str, camera_socket: Any, width: int, height: int) -> None:
        if self._camera_info_dims.get(slug) == (width, height) and slug in self._camera_info_cache:
            return
        if self._ffc_state is None or self._ffc_state.calib is None:
            return
        ci, fallback = build_camera_info_with_fallback(
            self._ffc_state.calib,
            camera_socket,
            width,
            height,
            self._frame_id_for_slug(slug),
        )
        self._camera_info_cache[slug] = ci
        self._camera_info_dims[slug] = (width, height)
        if fallback:
            if slug not in self._camera_info_fallback_logged:
                self._camera_info_fallback_logged.add(slug)
                self.get_logger().warn(
                    f'CameraInfo [{slug}]: no intrinsics for {width}x{height} on this device; '
                    'using placeholder K matrix. Recalibrate with Luxonis tools for real values.',
                )
        else:
            self.get_logger().info(f'CameraInfo [{slug}] {width}x{height} (from device calibration)')

    def _on_publish_tick(self) -> None:
        if self._ffc_state is None:
            return
        if self._ffc_state.error:
            self.get_logger().error(
                f'DepthAI error: {self._ffc_state.error}',
                throttle_duration_sec=5.0,
            )

        stamp = self.get_clock().now().to_msg()

        for camera_socket, slug in self._layout:
            with self._ffc_state.lock:
                buf = self._ffc_state.streams.get(slug)
                if buf is None:
                    continue
                rgb_frame = buf.rgb_frame
                img_dets = buf.img_detections

            if rgb_frame is None:
                continue

            try:
                bgr = rgb_frame.getCvFrame()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'[{slug}] getCvFrame failed: {e}', throttle_duration_sec=2.0)
                continue

            ih, iw = bgr.shape[:2]
            self._ensure_camera_info(slug, camera_socket, iw, ih)

            frame_id = self._frame_id_for_slug(slug)
            header = Header(stamp=stamp, frame_id=frame_id)

            img_msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            img_msg.header = header
            self._pub_image[slug].publish(img_msg)

            if slug in self._camera_info_cache:
                ci = self._camera_info_cache[slug]
                ci.header.stamp = stamp
                ci.header.frame_id = frame_id
                self._pub_cam_info[slug].publish(ci)

            if img_dets is not None:
                det2 = img_detections_to_detection2d_array(img_dets, header, iw, ih)
                self._pub_det2[slug].publish(det2)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FfcCameraNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.shutdown_resources()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
