"""ROS 2 Luxonis camera bridge: OAK-FFC-4P and/or OAK-D-Pro on one node."""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rover_msgs.msg import ImageDetection, ImageDetectionArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from luxonis_scripts.dai_ros_utils import (
    FFC_FRAME_IDS,
    NN_BASENAME,
    OAK_RGB_FRAME_ID,
    build_camera_info,
    build_camera_info_with_fallback,
    default_nn_archive_path,
    img_detections_to_rover,
    spatial_detections_to_rover,
)


def _ffc_socket_slug_pairs() -> list[tuple[dai.CameraBoardSocket, str]]:
    return [
        (dai.CameraBoardSocket.CAM_A, 'front'),
        (dai.CameraBoardSocket.CAM_B, 'right'),
        (dai.CameraBoardSocket.CAM_C, 'left'),
        (dai.CameraBoardSocket.CAM_D, 'back'),
    ]


def _filter_ffc_layout(active_cameras_param: str) -> list[tuple[dai.CameraBoardSocket, str]]:
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
    if watchdog_ms > 0:
        os.environ.setdefault('DEPTHAI_WATCHDOG', str(watchdog_ms))
    if watchdog_initial_delay_ms > 0:
        os.environ.setdefault('DEPTHAI_WATCHDOG_INITIAL_DELAY', str(watchdog_initial_delay_ms))
    if bootup_timeout_ms > 0:
        os.environ.setdefault('DEPTHAI_BOOTUP_TIMEOUT', str(bootup_timeout_ms))


def _resolve_device_info(device_mxid: str) -> dai.DeviceInfo:
    if device_mxid:
        return dai.DeviceInfo(device_mxid)
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        raise RuntimeError(
            'No DepthAI devices found. For PoE/Ethernet, pass the camera IP as '
            'device_mxid (launch: device_mxid:=10.x.x.x).'
        )
    return devices[0]


@dataclass
class FfcStreamBuffers:
    rgb_frame: Optional[dai.ImgFrame] = None
    img_detections: Optional[dai.ImgDetections] = None


@dataclass
class FfcSharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    calib: Optional[dai.CalibrationHandler] = None
    streams: dict[str, FfcStreamBuffers] = field(default_factory=dict)
    error: Optional[str] = None


@dataclass
class OakSharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    rgb_frame: Optional[dai.ImgFrame] = None
    depth_frame: Optional[dai.ImgFrame] = None
    spatial_detections: Optional[dai.ImgDetections] = None
    calib: Optional[dai.CalibrationHandler] = None
    error: Optional[str] = None


_OUTPUT_QUEUE_DEPTH = 16


class FfcQuadWorker(threading.Thread):
    def __init__(
        self,
        *,
        layout: list[tuple[dai.CameraBoardSocket, str]],
        device_mxid: str,
        nn_archive_path: str,
        fps: float,
        confidence_threshold: float,
        state: FfcSharedState,
        stop_event: threading.Event,
        logger: RcutilsLogger,
    ) -> None:
        super().__init__(daemon=True)
        self._layout = layout
        self._device_mxid = device_mxid.strip()
        self._nn_archive_path = nn_archive_path
        self._fps = fps
        self._confidence_threshold = confidence_threshold
        self._state = state
        self._stop_event = stop_event
        self._logger = logger

    def run(self) -> None:
        try:
            self._run_pipeline()
        except Exception as e:  # noqa: BLE001
            self._state.error = str(e)
            self._logger.error(f'DepthAI FFC pipeline failed: {e}')

    def _run_pipeline(self) -> None:
        device_info = _resolve_device_info(self._device_mxid)
        target = self._device_mxid or '(first discovered device)'
        self._logger.info(f'DepthAI opening FFC device {target!r} ({len(self._layout)} branch(es))...')
        device = dai.Device(device_info)

        layout = list(self._layout)
        calib = device.readCalibration()
        with self._state.lock:
            self._state.calib = calib
            self._state.streams = {slug: FfcStreamBuffers() for _, slug in layout}

        with dai.Pipeline(device) as pipeline:
            archive = dai.NNArchive(self._nn_archive_path)
            queues: list[tuple[str, dai.CameraBoardSocket, dai.DataOutputQueue, dai.DataOutputQueue]] = []
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
                    self._logger.error(
                        f'XLink / queue read failed (link overload or watchdog): {msg}. '
                        'Try lower fps, fewer cameras (active_cameras), or increase '
                        'depthai_watchdog_* parameters / DEPTHAI_WATCHDOG env.'
                    )
                    break
                time.sleep(0.001)


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
        ir_laser_dot_intensity: float,
        ir_flood_light_intensity: float,
        state: OakSharedState,
        stop_event: threading.Event,
        logger: RcutilsLogger,
    ) -> None:
        super().__init__(daemon=True)
        self._device_mxid = device_mxid.strip()
        self._nn_archive_path = nn_archive_path
        self._fps = fps
        self._confidence_threshold = confidence_threshold
        self._depth_lower_threshold = depth_lower_threshold
        self._depth_upper_threshold = depth_upper_threshold
        self._align_depth_to_rgb = align_depth_to_rgb
        self._ir_laser_dot_intensity = ir_laser_dot_intensity
        self._ir_flood_light_intensity = ir_flood_light_intensity
        self._state = state
        self._stop_event = stop_event
        self._logger = logger

    def run(self) -> None:
        try:
            self._run_pipeline()
        except Exception as e:  # noqa: BLE001
            self._state.error = str(e)
            self._logger.error(f'DepthAI OAK pipeline failed: {e}')

    def _run_pipeline(self) -> None:
        device_info = _resolve_device_info(self._device_mxid)
        target = self._device_mxid or '(first discovered device)'
        self._logger.info(f'DepthAI opening OAK device {target!r}...')
        device = dai.Device(device_info)
        try:
            device.setIrLaserDotProjectorIntensity(self._ir_laser_dot_intensity)
            device.setIrFloodLightIntensity(self._ir_flood_light_intensity)
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

    def _build_spatial_pipeline(
        self, pipeline: dai.Pipeline
    ) -> tuple[dai.DataOutputQueue, dai.DataOutputQueue, dai.DataOutputQueue]:
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
        if self._align_depth_to_rgb:
            try:
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            except Exception as e:  # noqa: BLE001
                self._logger.warn(
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
        depth_q = spatial_nn.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
        return rgb_q, det_q, depth_q


class LuxonisCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_camera')

        self.declare_parameter('ffc_device_mxid', '')
        self.declare_parameter('oak_device_mxid', '')
        self.declare_parameter('nn_archive_path', default_nn_archive_path())
        self.declare_parameter('ffc_fps', 4.0)
        self.declare_parameter('oak_fps', 15.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter(
            'active_cameras',
            'all',
            descriptor=ParameterDescriptor(
                description='FFC only: comma-separated slugs front,right,left,back or "all".',
            ),
        )
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('align_depth_to_rgb', True)
        self.declare_parameter('depth_lower_threshold', 200)
        self.declare_parameter('depth_upper_threshold', 8000)
        self.declare_parameter('ir_laser_dot_intensity', 1.0)
        self.declare_parameter('ir_flood_light_intensity', 0.5)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter(
            'depthai_watchdog_ms',
            4500,
            descriptor=ParameterDescriptor(description='PoE keepalive (ms); 0 leaves env unchanged.'),
        )
        self.declare_parameter('depthai_watchdog_initial_delay_ms', 60000)
        self.declare_parameter('depthai_bootup_timeout_ms', 60000)

        apply_depthai_poe_watchdog_defaults(
            watchdog_ms=int(self.get_parameter('depthai_watchdog_ms').get_parameter_value().integer_value),
            watchdog_initial_delay_ms=int(
                self.get_parameter('depthai_watchdog_initial_delay_ms').get_parameter_value().integer_value
            ),
            bootup_timeout_ms=int(
                self.get_parameter('depthai_bootup_timeout_ms').get_parameter_value().integer_value
            ),
        )

        self._publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._publish_depth = self.get_parameter('publish_depth').get_parameter_value().bool_value
        self._align_depth_to_rgb = self.get_parameter('align_depth_to_rgb').get_parameter_value().bool_value
        self._bridge = CvBridge()

        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub_image: dict[str, Publisher] = {}
        self._pub_cam_info: dict[str, Publisher] = {}
        self._depth_image_pub: Optional[Publisher] = None
        self._depth_cam_info_pub: Optional[Publisher] = None
        self._camera_info_cache: dict[str, CameraInfo] = {}
        self._camera_info_dims: dict[str, tuple[int, int]] = {}
        self._camera_info_fallback_logged: set[str] = set()
        self._oak_camera_info_msg: Optional[CameraInfo] = None
        self._oak_camera_info_dims: Optional[tuple[int, int]] = None
        self._depth_camera_info_msg: Optional[CameraInfo] = None
        self._depth_camera_info_dims: Optional[tuple[int, int]] = None

        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self._detections_pub = self.create_publisher(ImageDetectionArray, detections_topic, 10)

        self._ffc_layout: list[tuple[dai.CameraBoardSocket, str]] = []
        self._ffc_state: Optional[FfcSharedState] = None
        self._ffc_stop: Optional[threading.Event] = None
        self._ffc_thread: Optional[FfcQuadWorker] = None

        self._oak_state: Optional[OakSharedState] = None
        self._oak_stop: Optional[threading.Event] = None
        self._oak_thread: Optional[OakSpatialWorker] = None

        ffc_mxid = self.get_parameter('ffc_device_mxid').get_parameter_value().string_value.strip()
        oak_mxid = self.get_parameter('oak_device_mxid').get_parameter_value().string_value.strip()
        nn_path = self._resolve_nn_path()

        if ffc_mxid:
            active_raw = self.get_parameter('active_cameras').get_parameter_value().string_value
            try:
                self._ffc_layout = _filter_ffc_layout(active_raw)
            except ValueError as e:
                self.get_logger().error(str(e))
            else:
                for _sock, slug in self._ffc_layout:
                    self._pub_image[slug] = self.create_publisher(
                        Image, f'{slug}/image_raw', qos_profile_sensor_data
                    )
                    self._pub_cam_info[slug] = self.create_publisher(
                        CameraInfo, f'{slug}/camera_info', cam_qos
                    )
                self._start_ffc(ffc_mxid, nn_path)

        if oak_mxid:
            self._pub_image['rgb'] = self.create_publisher(Image, 'rgb/image_raw', qos_profile_sensor_data)
            self._pub_cam_info['rgb'] = self.create_publisher(CameraInfo, 'rgb/camera_info', cam_qos)
            if self._publish_depth:
                self._depth_image_pub = self.create_publisher(
                    Image, 'depth/image_raw', qos_profile_sensor_data
                )
                self._depth_cam_info_pub = self.create_publisher(
                    CameraInfo, 'depth/camera_info', cam_qos
                )
            self._start_oak(oak_mxid, nn_path)

        if not ffc_mxid and not oak_mxid:
            self.get_logger().error('Set ffc_device_mxid and/or oak_device_mxid to start a pipeline.')

        period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.033
        self._timer = self.create_timer(period, self._on_publish_tick)

        self.get_logger().info(
            f'Luxonis camera node started ffc={ffc_mxid!r} oak={oak_mxid!r} '
            f'detections_topic={detections_topic!r}'
        )

    def _resolve_nn_path(self) -> str:
        nn_path = self.get_parameter('nn_archive_path').get_parameter_value().string_value.strip()
        if not nn_path:
            nn_path = default_nn_archive_path()
        if not nn_path or not Path(nn_path).is_file():
            self.get_logger().error(
                f'NN archive not found: {(nn_path or "empty")!r}. '
                'Set nn_archive_path or LUXONIS_NN_ARCHIVE, or install the model under '
                f'share/luxonis_scripts/models/{NN_BASENAME}.'
            )
            return ''
        return nn_path

    def _start_ffc(self, device_mxid: str, nn_path: str) -> None:
        if not self._ffc_layout or not nn_path:
            return
        self._ffc_stop = threading.Event()
        self._ffc_state = FfcSharedState()
        self._ffc_thread = FfcQuadWorker(
            layout=list(self._ffc_layout),
            device_mxid=device_mxid,
            nn_archive_path=nn_path,
            fps=float(self.get_parameter('ffc_fps').get_parameter_value().double_value),
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').get_parameter_value().double_value
            ),
            state=self._ffc_state,
            stop_event=self._ffc_stop,
            logger=self.get_logger(),
        )
        self._ffc_thread.start()

    def _start_oak(self, device_mxid: str, nn_path: str) -> None:
        if not nn_path:
            return
        self._oak_stop = threading.Event()
        self._oak_state = OakSharedState()
        self._oak_thread = OakSpatialWorker(
            device_mxid=device_mxid,
            nn_archive_path=nn_path,
            fps=float(self.get_parameter('oak_fps').get_parameter_value().double_value),
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
            ir_laser_dot_intensity=float(
                self.get_parameter('ir_laser_dot_intensity').get_parameter_value().double_value
            ),
            ir_flood_light_intensity=float(
                self.get_parameter('ir_flood_light_intensity').get_parameter_value().double_value
            ),
            state=self._oak_state,
            stop_event=self._oak_stop,
            logger=self.get_logger(),
        )
        self._oak_thread.start()

    def shutdown_resources(self) -> None:
        if self._ffc_stop is not None:
            self._ffc_stop.set()
        if self._ffc_thread is not None:
            self._ffc_thread.join(timeout=30.0)
        if self._oak_stop is not None:
            self._oak_stop.set()
        if self._oak_thread is not None:
            self._oak_thread.join(timeout=8.0)

    def _frame_id_for_slug(self, slug: str) -> str:
        return FFC_FRAME_IDS[slug]

    def _ensure_ffc_camera_info(
        self, slug: str, camera_socket: dai.CameraBoardSocket, width: int, height: int
    ) -> None:
        if self._camera_info_dims.get(slug) == (width, height) and slug in self._camera_info_cache:
            return
        if self._ffc_state is None or self._ffc_state.calib is None:
            return
        frame_id = self._frame_id_for_slug(slug)
        ci, fallback = build_camera_info_with_fallback(
            self._ffc_state.calib, camera_socket, width, height, frame_id
        )
        self._camera_info_cache[slug] = ci
        self._camera_info_dims[slug] = (width, height)
        if fallback and slug not in self._camera_info_fallback_logged:
            self._camera_info_fallback_logged.add(slug)
            self.get_logger().warn(
                f'CameraInfo [{slug}]: no intrinsics for {width}x{height}; using placeholder K matrix.'
            )

    def _ensure_oak_camera_info(self, width: int, height: int) -> None:
        if self._oak_camera_info_dims == (width, height) and self._oak_camera_info_msg is not None:
            return
        if self._oak_state is None or self._oak_state.calib is None:
            return
        self._oak_camera_info_msg = build_camera_info(
            self._oak_state.calib,
            dai.CameraBoardSocket.CAM_A,
            width,
            height,
            OAK_RGB_FRAME_ID,
        )
        self._oak_camera_info_dims = (width, height)

    def _ensure_depth_camera_info(self, width: int, height: int) -> None:
        if self._depth_camera_info_dims == (width, height) and self._depth_camera_info_msg is not None:
            return
        if self._oak_state is None or self._oak_state.calib is None:
            return
        socket = (
            dai.CameraBoardSocket.CAM_A
            if self._align_depth_to_rgb
            else dai.CameraBoardSocket.CAM_C
        )
        self._depth_camera_info_msg = build_camera_info(
            self._oak_state.calib,
            socket,
            width,
            height,
            OAK_RGB_FRAME_ID,
        )
        self._depth_camera_info_dims = (width, height)

    def _on_publish_tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        all_detections: list[ImageDetection] = []

        if self._ffc_state is not None:
            if self._ffc_state.error:
                self.get_logger().error(
                    f'FFC DepthAI error: {self._ffc_state.error}',
                    throttle_duration_sec=5.0,
                )
            self._publish_ffc(stamp, all_detections)

        if self._oak_state is not None:
            if self._oak_state.error:
                self.get_logger().error(
                    f'OAK DepthAI error: {self._oak_state.error}',
                    throttle_duration_sec=5.0,
                )
            self._publish_oak(stamp, all_detections)

        if all_detections:
            det_msg = ImageDetectionArray()
            det_msg.detections = all_detections
            self._detections_pub.publish(det_msg)

    def _publish_ffc(self, stamp, all_detections: list[ImageDetection]) -> None:
        assert self._ffc_state is not None
        for camera_socket, slug in self._ffc_layout:
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
            self._ensure_ffc_camera_info(slug, camera_socket, iw, ih)
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
                all_detections.extend(img_detections_to_rover(img_dets, header, iw, ih))

    def _publish_oak(self, stamp, all_detections: list[ImageDetection]) -> None:
        assert self._oak_state is not None
        with self._oak_state.lock:
            rgb_frame = self._oak_state.rgb_frame
            depth_frame = self._oak_state.depth_frame
            spatial_dets = self._oak_state.spatial_detections

        if rgb_frame is None:
            return

        try:
            bgr = rgb_frame.getCvFrame()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'OAK getCvFrame failed: {e}', throttle_duration_sec=2.0)
            return

        h, w = bgr.shape[:2]
        self._ensure_oak_camera_info(w, h)
        header = Header(stamp=stamp, frame_id=OAK_RGB_FRAME_ID)

        img_msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        img_msg.header = header
        self._pub_image['rgb'].publish(img_msg)

        if self._oak_camera_info_msg is not None:
            ci = self._oak_camera_info_msg
            ci.header.stamp = stamp
            ci.header.frame_id = OAK_RGB_FRAME_ID
            self._pub_cam_info['rgb'].publish(ci)

        if self._publish_depth and depth_frame is not None and self._depth_image_pub is not None:
            self._publish_depth_frame(depth_frame, header, stamp)

        if spatial_dets is not None:
            all_detections.extend(spatial_detections_to_rover(spatial_dets, header, w, h))

    def _publish_depth_frame(self, depth_frame: dai.ImgFrame, header: Header, stamp) -> None:
        assert self._depth_image_pub is not None
        try:
            depth = depth_frame.getCvFrame()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'depth getCvFrame failed: {e}', throttle_duration_sec=2.0)
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

        if self._depth_cam_info_pub is not None and self._depth_camera_info_msg is not None:
            dci = self._depth_camera_info_msg
            dci.header.stamp = stamp
            dci.header.frame_id = OAK_RGB_FRAME_ID
            self._depth_cam_info_pub.publish(dci)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LuxonisCameraNode()
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
