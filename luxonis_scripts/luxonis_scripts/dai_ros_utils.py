"""Shared DepthAI helpers for ROS 2: NN archive lookup, CameraInfo, rover_msgs conversions."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Sequence

import depthai as dai
from geometry_msgs.msg import Point
from rover_msgs.msg import ImageDetection
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

NN_BASENAME = 'best_190_Epoch.rvc2.tar.xz'

# rover-description/urdf/body/ffc-mount.urdf
FFC_FRAME_IDS: dict[str, str] = {
    'front': 'ffc_front_camera',
    'right': 'ffc_right_camera',
    'left': 'ffc_left_camera',
    'back': 'ffc_rear_camera',
}

# rover-description/urdf/body/autonomy-module.urdf (depth camera link TBD in depth-camera.urdf)
OAK_RGB_FRAME_ID = 'forward_camera'


def default_nn_archive_path() -> str:
    env = os.environ.get('LUXONIS_NN_ARCHIVE', '').strip()
    if env and Path(env).is_file():
        return env
    try:
        from ament_index_python.packages import get_package_share_directory

        cand = Path(get_package_share_directory('luxonis_scripts')) / 'models' / NN_BASENAME
        if cand.is_file():
            return str(cand)
    except Exception:  # noqa: BLE001
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / NN_BASENAME
        if cand.is_file():
            return str(cand)
    return ''


def build_camera_info(
    calib: dai.CalibrationHandler,
    camera_socket: dai.CameraBoardSocket,
    width: int,
    height: int,
    frame_id: str,
) -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height

    k_mat = calib.getCameraIntrinsics(camera_socket, (width, height))
    msg.k = [
        k_mat[0][0],
        k_mat[0][1],
        k_mat[0][2],
        k_mat[1][0],
        k_mat[1][1],
        k_mat[1][2],
        k_mat[2][0],
        k_mat[2][1],
        k_mat[2][2],
    ]

    d_list: Sequence[float] = calib.getDistortionCoefficients(camera_socket)
    msg.d = list(d_list)

    ld = len(msg.d)
    if ld <= 5:
        msg.distortion_model = 'plumb_bob'
    elif ld >= 8:
        msg.distortion_model = 'rational_polynomial'
    else:
        msg.distortion_model = 'plumb_bob'

    model = calib.getDistortionModel(camera_socket)
    if model == dai.CameraModel.Fisheye:
        msg.distortion_model = 'equidistant'

    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    fx, fy = msg.k[0], msg.k[4]
    cx, cy = msg.k[2], msg.k[5]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    msg.binning_x = 0
    msg.binning_y = 0
    return msg


def build_placeholder_camera_info(width: int, height: int, frame_id: str) -> CameraInfo:
    """Rough pinhole when the device has no intrinsics at this resolution (common on FFC / legacy cal)."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    fx = float(max(width, height))
    fy = fx
    cx = float(width) * 0.5
    cy = float(height) * 0.5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.d = []
    msg.distortion_model = 'plumb_bob'
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    return msg


def build_camera_info_with_fallback(
    calib: dai.CalibrationHandler,
    camera_socket: dai.CameraBoardSocket,
    width: int,
    height: int,
    frame_id: str,
) -> tuple[CameraInfo, bool]:
    """Build ``CameraInfo`` from device calibration, or a placeholder if intrinsics are missing."""
    try:
        return (build_camera_info(calib, camera_socket, width, height, frame_id), False)
    except Exception:
        return (build_placeholder_camera_info(width, height, frame_id), True)


def norm_xy_to_px(value: float, extent: int) -> float:
    """DepthAI reports box edges normalized to [0, 1] relative to image width/height."""
    return float(value * extent)


def img_detections_to_rover(
    src: dai.ImgDetections,
    header: Header,
    image_width: int,
    image_height: int,
) -> list[ImageDetection]:
    out: list[ImageDetection] = []
    w, h = image_width, image_height
    for d in src.detections:
        det = ImageDetection()
        det.header = header
        det.label = d.labelName or str(d.label)
        det.confidence = float(d.confidence)
        det.min = Point(
            x=norm_xy_to_px(d.xmin, w),
            y=norm_xy_to_px(d.ymin, h),
            z=0.0,
        )
        det.max = Point(
            x=norm_xy_to_px(d.xmax, w),
            y=norm_xy_to_px(d.ymax, h),
            z=0.0,
        )
        out.append(det)
    return out


def spatial_detections_to_rover(
    src: dai.ImgDetections,
    header: Header,
    image_width: int,
    image_height: int,
) -> list[ImageDetection]:
    """Same 2D bbox layout as :func:`img_detections_to_rover` (DepthAI normalized coords)."""
    out: list[ImageDetection] = []
    w, h = image_width, image_height
    for d in src.detections:
        det = ImageDetection()
        det.header = header
        det.label = d.labelName or str(d.label)
        det.confidence = float(d.confidence)
        det.min = Point(
            x=norm_xy_to_px(d.xmin, w),
            y=norm_xy_to_px(d.ymin, h),
            z=0.0,
        )
        det.max = Point(
            x=norm_xy_to_px(d.xmax, w),
            y=norm_xy_to_px(d.ymax, h),
            z=0.0,
        )
        out.append(det)
    return out
