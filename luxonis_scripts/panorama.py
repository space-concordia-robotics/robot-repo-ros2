"""360 panorama builder for the FFC kit.

Takes one snapshot per camera (FRONT, RIGHT, LEFT, BACK), tries OpenCV's
Stitcher for a seamless result, falls back to horizontal concatenation when
the scene lacks enough overlap texture, and overlays a cardinal-direction
strip rotated by the IMU yaw.

Two usage modes:

1. Library: ``build_and_save(snaps, imu_pkt, out_dir, hfov_deg=108.0)``
   Stitch from already-captured BGR frames.

2. Standalone subprocess (what ``main.py``'s ``panorama`` REPL command
   invokes after it stops the live streams): ``python3 panorama.py``
   opens the FFC and the OAK-D directly, grabs one frame per camera +
   one IMU packet, and stitches.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np


FFC_MXID_DEFAULT = "14442C10014791D700"
OAKD_MXID_DEFAULT = "1944301001EDE12E00"
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(__file__), "panoramas")


CLOCKWISE_ORDER = ("FRONT", "RIGHT", "BACK", "LEFT")
CARDINALS = [
    (0.0, "N"),
    (45.0, "NE"),
    (90.0, "E"),
    (135.0, "SE"),
    (180.0, "S"),
    (225.0, "SW"),
    (270.0, "W"),
    (315.0, "NW"),
]


def build_and_save(snaps, imu_pkt, out_dir, hfov_deg=108.0, filename=None):
    """Build a panorama from 4 FFC snapshots and save it.

    snaps: dict like {"FRONT": bgr, "RIGHT": bgr, "BACK": bgr, "LEFT": bgr}
    imu_pkt: latest dai.IMUData packet (or None to use yaw=0)
    out_dir: Path or str of the directory to write into
    hfov_deg: per-camera horizontal FOV (108 for IMX378-W default)
    filename: optional explicit name (no extension needed); auto-timestamped otherwise

    Returns: Path of the written file.
    """
    missing = [c for c in CLOCKWISE_ORDER if snaps.get(c) is None]
    if missing:
        raise ValueError(f"missing snapshots for: {','.join(missing)}")

    frames = [snaps[c] for c in CLOCKWISE_ORDER]
    yaw_deg = _yaw_from_imu(imu_pkt)

    pano, stitched = _stitch(frames)
    pano = _overlay_compass(pano, yaw_deg, hfov_deg, stitched)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if filename is None:
        filename = f"panorama_{time.strftime('%Y%m%d_%H%M%S')}"
    if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
        filename += ".png"
    out_path = out_dir / filename
    cv2.imwrite(str(out_path), pano)
    return out_path


# ---------- IMU ----------

def _yaw_from_imu(imu_pkt):
    """Return yaw in [0, 360) degrees, or 0.0 if no IMU data."""
    if imu_pkt is None or not getattr(imu_pkt, "packets", None):
        return 0.0
    pkt = imu_pkt.packets[-1]
    rv = getattr(pkt, "rotationVector", None)
    if rv is not None and (rv.i or rv.j or rv.k or rv.real):
        return _yaw_from_quat(rv.i, rv.j, rv.k, rv.real)
    mag = getattr(pkt, "magneticField", None)
    if mag is not None and (mag.x or mag.y):
        return (math.degrees(math.atan2(mag.y, mag.x)) + 360.0) % 360.0
    return 0.0


def _yaw_from_quat(x, y, z, w):
    """Yaw (rotation about Z) from a unit quaternion, in degrees [0, 360)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return (yaw + 360.0) % 360.0


# ---------- Stitching ----------

def _stitch(frames):
    """Return (image, stitched_bool). stitched=True if OpenCV produced a 360
    seamless image; False if we fell back to plain concatenation."""
    try:
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(frames)
        if status == cv2.Stitcher_OK and pano is not None and pano.size > 0:
            return pano, True
    except Exception:
        pass
    return _concat(frames), False


def _concat(frames):
    """Plain horizontal concat with thin seam markers."""
    target_h = min(f.shape[0] for f in frames)
    resized = []
    for f in frames:
        if f.shape[0] != target_h:
            scale = target_h / f.shape[0]
            new_w = int(f.shape[1] * scale)
            resized.append(cv2.resize(f, (new_w, target_h)))
        else:
            resized.append(f)
    pano = np.hstack(resized)
    x = 0
    for f in resized[:-1]:
        x += f.shape[1]
        cv2.line(pano, (x, 0), (x, pano.shape[0]), (0, 0, 255), 1)
    return pano


# ---------- Compass overlay ----------

def _overlay_compass(img, yaw_deg, hfov_deg, stitched):
    """Draw a cardinal compass strip across the top of img.

    yaw_deg = robot heading in world frame (0 = North, 90 = East).
    The robot's FRONT camera centerline points along yaw_deg.

    For the concat case the panorama spans 4 * hfov_deg horizontally; for
    stitched it spans 360 deg.
    """
    h, w = img.shape[:2]
    strip_h = max(48, h // 14)
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, strip_h), (0, 0, 0), -1)
    img = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)

    cv2.line(img, (0, strip_h), (w, strip_h), (255, 255, 255), 1)

    for tick_deg in range(0, 360, 15):
        for x in _angle_to_xs(tick_deg, yaw_deg, w, hfov_deg, stitched):
            length = 14 if tick_deg % 90 == 0 else (10 if tick_deg % 45 == 0 else 6)
            cv2.line(img, (x, strip_h - length), (x, strip_h), (255, 255, 255), 1)

    for world_deg, label in CARDINALS:
        major = label in ("N", "E", "S", "W")
        font_scale = 0.9 if major else 0.5
        color = (0, 220, 255) if major else (200, 200, 200)
        thickness = 2 if major else 1
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        for x in _angle_to_xs(world_deg, yaw_deg, w, hfov_deg, stitched):
            tx = max(0, min(w - tw, x - tw // 2))
            ty = strip_h - 18 if major else strip_h - 14
            cv2.putText(img, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, color, thickness, cv2.LINE_AA)

    yaw_text = f"yaw {yaw_deg:5.1f}deg  hfov {hfov_deg:.0f}  {'stitched' if stitched else 'concat'}"
    cv2.putText(img, yaw_text, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def _angle_to_xs(world_deg, yaw_deg, width, hfov_deg, stitched):
    """Pixel column(s) where a given world heading appears on the panorama.

    Returns a list (possibly empty, possibly 2 entries in the concat case if
    the angle lies in an overlap zone).
    """
    rel = (world_deg - yaw_deg + 360.0) % 360.0  # clockwise from FRONT center
    if stitched:
        x = int(rel / 360.0 * width)
        return [max(0, min(width - 1, x))]

    cam_w = width / 4.0
    xs = []
    for i in range(4):
        center = i * 90.0
        d = ((rel - center + 540.0) % 360.0) - 180.0
        if -hfov_deg / 2.0 <= d <= hfov_deg / 2.0:
            frac = (d + hfov_deg / 2.0) / hfov_deg
            x = int(i * cam_w + frac * cam_w)
            xs.append(max(0, min(width - 1, x)))
    return xs


# ---------- Standalone capture (subprocess entry-point) ----------

def _capture_ffc_frames(device_id, warmup_s=10.0, capture_timeout_s=6.0,
                        fps=10, resolution=(1920, 1080)):
    """Open the FFC and grab one BGR frame per directional camera.

    Used by ``main.py`` after it has stopped the live streaming pipeline.
    We open the device cleanly here (no encoders, no visualizer) so the
    capture takes ~3-4 s end to end.
    """
    import depthai as dai

    sockets = {
        "FRONT": dai.CameraBoardSocket.CAM_A,
        "RIGHT": dai.CameraBoardSocket.CAM_B,
        "LEFT":  dai.CameraBoardSocket.CAM_C,
        "BACK":  dai.CameraBoardSocket.CAM_D,
    }

    snaps = {}
    device = dai.Device(dai.DeviceInfo(device_id))
    with device, dai.Pipeline(device) as pipeline:
        queues = {}
        for name, sock in sockets.items():
            cam = pipeline.create(dai.node.Camera).build(sock)
            cam_out = cam.requestOutput(resolution, fps=fps,
                                        type=dai.ImgFrame.Type.NV12)
            queues[name] = cam_out.createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()

        # Drain the warmup window so autoexposure / autowhitebalance settle.
        warmup_end = time.monotonic() + 10.0
        while time.monotonic() < warmup_end:
            for q in queues.values():
                while q.has():
                    q.get()
            time.sleep(0.1)

        deadline = time.monotonic() + capture_timeout_s
        while len(snaps) < len(sockets) and time.monotonic() < deadline:
            for name, q in queues.items():
                if name in snaps:
                    continue
                if q.has():
                    frame = q.get().getCvFrame()
                    if frame is not None and frame.size > 0:
                        snaps[name] = frame
            time.sleep(0.1)

    missing = [n for n in sockets if n not in snaps]
    if missing:
        raise RuntimeError(f"failed to capture FFC frames for: {','.join(missing)}")
    return snaps


def _capture_imu_packet(device_id, timeout_s=4.0):
    """Open the OAK-D briefly to grab one IMU packet (rotation vector + mag).

    Returns the latest packet, or None on any failure. The caller treats
    None as 'no orientation available' and the compass overlay falls
    back to yaw=0.
    """
    try:
        import depthai as dai
    except Exception as e:
        print(f"[panorama] depthai import failed for IMU: {e}", file=sys.stderr)
        return None

    try:
        device = dai.Device(dai.DeviceInfo(device_id))
    except Exception as e:
        print(f"[panorama] could not open OAK-D for IMU ({device_id}): {e}",
              file=sys.stderr)
        return None

    try:
        with device, dai.Pipeline(device) as pipeline:
            imu = pipeline.create(dai.node.IMU)
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 50)
            try:
                imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_CALIBRATED, 50)
            except Exception:
                pass
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
            imu_q = imu.out.createOutputQueue(maxSize=10, blocking=False)
            pipeline.start()

            deadline = time.monotonic() + timeout_s
            last_pkt = None
            while time.monotonic() < deadline:
                if imu_q.has():
                    pkt = imu_q.get()
                    last_pkt = pkt
                    if pkt is not None and pkt.packets:
                        rv = getattr(pkt.packets[-1], "rotationVector", None)
                        if rv is not None and (rv.i or rv.j or rv.k or rv.real):
                            return pkt
                time.sleep(0.02)
            return last_pkt
    except Exception as e:
        print(f"[panorama] OAK-D IMU capture failed: {e}", file=sys.stderr)
        return None


def _parse_cli_args():
    p = argparse.ArgumentParser(description="Stitch a 360 panorama from the FFC + OAK-D IMU")
    p.add_argument("--ffc", default=FFC_MXID_DEFAULT,
                   help="FFC MxId or IP (default: %(default)s)")
    p.add_argument("--oakd", default=OAKD_MXID_DEFAULT,
                   help="OAK-D MxId or IP for the IMU packet (default: %(default)s)")
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR,
                   help="Directory to write the panorama into (default: %(default)s)")
    p.add_argument("--filename", default=None,
                   help="Output filename (auto-timestamped if omitted)")
    p.add_argument("--hfov-deg", type=float, default=108.0,
                   help="Per-camera horizontal FOV in degrees (default: %(default)s)")
    p.add_argument("--no-imu", action="store_true",
                   help="Skip the OAK-D IMU capture and use yaw=0")
    return p.parse_args()


def _main():
    args = _parse_cli_args()
    print(f"[panorama] capturing FFC frames from {args.ffc}...")
    try:
        snaps = _capture_ffc_frames(args.ffc)
    except Exception as e:
        print(f"[panorama] FFC capture failed: {e}", file=sys.stderr)
        return 2

    imu_pkt = None
    if not args.no_imu:
        print(f"[panorama] capturing OAK-D IMU from {args.oakd}...")
        imu_pkt = _capture_imu_packet(args.oakd)
        if imu_pkt is None:
            print("[panorama] no IMU data; compass overlay will use yaw=0")

    print("[panorama] stitching...")
    out_path = build_and_save(snaps, imu_pkt, args.out_dir,
                              hfov_deg=args.hfov_deg, filename=args.filename)
    print(f"[panorama] saved: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
