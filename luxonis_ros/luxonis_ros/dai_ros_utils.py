"""Shared DepthAI helpers for ROS 2: NN archive lookup, CameraInfo, vision_msgs conversions."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Sequence

from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Pose2D,
)

NN_BASENAME = 'best_190_Epoch.rvc2.tar.xz'


def default_nn_archive_path() -> str:
    env = os.environ.get('LUXONIS_NN_ARCHIVE', '').strip()
    if env and Path(env).is_file():
        return env
    try:
        from ament_index_python.packages import get_package_share_directory

        cand = Path(get_package_share_directory('luxonis_ros')) / 'models' / NN_BASENAME
        if cand.is_file():
            return str(cand)
    except Exception:  # noqa: BLE001
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / 'luxonis_scripts' / NN_BASENAME
        if cand.is_file():
            return str(cand)
    return ''


def build_camera_info(
    calib: Any,
    camera_socket: Any,
    width: int,
    height: int,
    frame_id: str,
) -> CameraInfo:
    import depthai as dai_

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
    if model == dai_.CameraModel.Fisheye:
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
    calib: Any,
    camera_socket: Any,
    width: int,
    height: int,
    frame_id: str,
) -> tuple[CameraInfo, bool]:
    """Build ``CameraInfo`` from device calibration, or a placeholder if intrinsics are missing."""
    try:
        return (build_camera_info(calib, camera_socket, width, height, frame_id), False)
    except Exception:
        return (build_placeholder_camera_info(width, height, frame_id), True)


def identity_orientation() -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def norm_xy_to_px(v: float, extent: int) -> float:
    if 0.0 <= v <= 1.0:
        return float(v * extent)
    return float(v)


def hypothesis_with_pose(
    label: str,
    score: float,
    px: float,
    py: float,
    pz: float,
) -> ObjectHypothesisWithPose:
    hyp = ObjectHypothesis(class_id=label, score=float(score))
    pose = Pose(position=Point(x=px, y=py, z=pz), orientation=identity_orientation())
    ohp = ObjectHypothesisWithPose()
    ohp.hypothesis = hyp
    ohp.pose = PoseWithCovariance()
    ohp.pose.pose = pose
    ohp.pose.covariance = [0.0] * 36
    return ohp


def img_detections_to_detection2d_array(
    src: Any,
    header: Header,
    image_width: int,
    image_height: int,
) -> Detection2DArray:
    """Convert DepthAI ImgDetections to vision_msgs/Detection2DArray (image coordinates, pose unused)."""
    out = Detection2DArray(header=header)
    w, h = image_width, image_height
    for d in src.detections:
        label = d.labelName or str(d.label)
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
        ohp = hypothesis_with_pose(label, d.confidence, 0.0, 0.0, 0.0)
        det2 = Detection2D()
        det2.header = header
        det2.results = [ohp]
        det2.id = label
        det2.bbox = bb2
        out.detections.append(det2)
    return out
