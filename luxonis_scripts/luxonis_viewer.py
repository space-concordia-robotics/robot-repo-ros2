#!/usr/bin/env python3
"""Tiled RTSP client for luxonis_encode_gstreamer.py (optional YOLO).

Start the encoder first. Default --rtsp-backend auto uses FFmpeg if GStreamer
is unavailable (common with pip opencv-python). Many streams need enough
network bandwidth from the cameras to the PC — lower encoder --bitrate if video corrupts.
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

FFC_STREAMS = ["ffc0", "ffc1", "ffc2", "ffc3"]
OAKD_STREAMS = ["oakd_rgb", "oakd_left", "oakd_right"]

_rtsp_wait_detail_lock = threading.Lock()
_rtsp_wait_detail_printed = False


def streams_for_sources(sources: str) -> List[str]:
    if sources == "ffc":
        return list(FFC_STREAMS)
    if sources == "oakd":
        return list(OAKD_STREAMS)
    return list(FFC_STREAMS) + list(OAKD_STREAMS)


def rtsp_url(host: str, name: str) -> str:
    return f"rtsp://{host}:8554/{name}"


def encoder_tcp_listening(host: str, port: int = 8554, timeout: float = 0.5) -> bool:
    """Return True if host:port accepts TCP (encoder RTSP server)."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def gstreamer_pipeline(url: str, latency_ms: int = 200) -> str:
    return (
        f"rtspsrc location={url} latency={latency_ms} ! "
        "rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true sync=false max-buffers=1"
    )


def open_rtsp_capture(url: str, backend: str):
    """Open RTSP URL; return (cap, backend_name) or (None, None)."""
    if backend in ("auto", "gstreamer"):
        cap = cv2.VideoCapture(gstreamer_pipeline(url), cv2.CAP_GSTREAMER)
        if cap.isOpened():
            return cap, "gstreamer"
        cap.release()
        if backend == "gstreamer":
            return None, None
    if backend in ("auto", "ffmpeg"):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            return cap, "ffmpeg"
        cap.release()
    return None, None


class FrameReader(threading.Thread):
    def __init__(self, name: str, url: str, backend: str = "auto"):
        super().__init__(daemon=True)
        self.name = name
        self.url = url
        self.backend = backend
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running = True

    def stop(self) -> None:
        self._running = False

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def run(self) -> None:
        global _rtsp_wait_detail_printed
        cap = None
        used = None
        n_try = 0
        while self._running and cap is None:
            cap, used = open_rtsp_capture(self.url, self.backend)
            if cap is None:
                n_try += 1
                if n_try == 1:
                    with _rtsp_wait_detail_lock:
                        if not _rtsp_wait_detail_printed:
                            _rtsp_wait_detail_printed = True
                            print(
                                f"[{self.name}] waiting for RTSP ({self.url}).\n"
                                "    Run: python3 luxonis_scripts/luxonis_encode_gstreamer.py\n"
                                "    If the encoder is already up: try --rtsp-backend ffmpeg"
                            )
                elif n_try % 20 == 0:
                    print(f"[{self.name}] still waiting for RTSP …")
                time.sleep(1.0)
        if not self._running:
            if cap is not None:
                cap.release()
            return
        if self.backend == "auto":
            print(f"[{self.name}] opened via {used}")
        elif self.backend == "ffmpeg":
            print(f"[{self.name}] opened via ffmpeg")
        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
        cap.release()


def resize_cell(frame: np.ndarray, cell_w: int, cell_h: int) -> np.ndarray:
    return cv2.resize(frame, (cell_w, cell_h), interpolation=cv2.INTER_AREA)


def black_cell(cell_w: int, cell_h: int, label: str) -> np.ndarray:
    img = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
    cv2.putText(
        img,
        label,
        (10, cell_h // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (128, 128, 128),
        1,
        cv2.LINE_AA,
    )
    return img


def draw_label_bar(img: np.ndarray, text: str, show: bool) -> None:
    if not show:
        return
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 28), (0, 0, 0), -1)
    cv2.putText(
        img,
        text,
        (6, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def extract_boxes(results, model, min_conf: float):
    boxes = []
    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf < min_conf:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            label = model.names[cls]
            boxes.append((x1, y1, x2, y2, label, conf))
    return boxes


def draw_boxes(
    frame: np.ndarray, boxes: List[Tuple[int, int, int, int, str, float]]
) -> None:
    for x1, y1, x2, y2, label, conf in boxes:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"{label} {conf:.2f}",
            (x1, max(y1 - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )


class YoloRunner:
    def __init__(
        self,
        model_path: str,
        min_conf: float,
        stride: int,
        imgsz: int,
        half: bool,
        yolo_rgb_only: bool,
    ):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.min_conf = min_conf
        self.stride = max(1, stride)
        self.imgsz = imgsz
        self.half = half
        self.yolo_rgb_only = yolo_rgb_only
        self._counts: Dict[str, int] = {}
        self._cache: Dict[str, List[Tuple[int, int, int, int, str, float]]] = {}

    def should_run(self, stream_name: str) -> bool:
        if self.yolo_rgb_only and stream_name != "oakd_rgb":
            return False
        return True

    def annotate(self, stream_name: str, frame: np.ndarray) -> np.ndarray:
        if not self.should_run(stream_name):
            return frame
        c = self._counts.get(stream_name, 0)
        self._counts[stream_name] = c + 1
        if c % self.stride != 0:
            draw_boxes(frame, self._cache.get(stream_name, []))
            return frame
        kwargs = {"verbose": False, "imgsz": self.imgsz}
        if self.half:
            kwargs["half"] = True
        results = self.model(frame, **kwargs)
        boxes = extract_boxes(results, self.model, self.min_conf)
        self._cache[stream_name] = boxes
        draw_boxes(frame, boxes)
        return frame


def build_grid(
    sources: str,
    frames: Dict[str, np.ndarray],
    cell_w: int,
    cell_h: int,
    show_labels: bool,
) -> np.ndarray:
    def prep(name: str) -> np.ndarray:
        f = frames.get(name)
        if f is None:
            return black_cell(cell_w, cell_h, f"{name}: no signal")
        out = resize_cell(f, cell_w, cell_h)
        draw_label_bar(out, name, show_labels)
        return out

    if sources == "ffc":
        row0 = np.hstack([prep("ffc0"), prep("ffc1")])
        row1 = np.hstack([prep("ffc2"), prep("ffc3")])
        return np.vstack([row0, row1])

    if sources == "oakd":
        return np.hstack([prep("oakd_rgb"), prep("oakd_left"), prep("oakd_right")])

    row0 = np.hstack([prep("ffc0"), prep("ffc1")])
    row1 = np.hstack([prep("ffc2"), prep("ffc3")])
    ffc_block = np.vstack([row0, row1])
    oakd_row = np.hstack([prep("oakd_rgb"), prep("oakd_left"), prep("oakd_right")])
    w_top = ffc_block.shape[1]
    w_bot = oakd_row.shape[1]
    target_w = max(w_top, w_bot)
    if w_top < target_w:
        pad = target_w - w_top
        ffc_block = np.hstack(
            [ffc_block, np.zeros((ffc_block.shape[0], pad, 3), dtype=np.uint8)]
        )
    if w_bot < target_w:
        pad = target_w - w_bot
        oakd_row = np.hstack(
            [oakd_row, np.zeros((oakd_row.shape[0], pad, 3), dtype=np.uint8)]
        )
    return np.vstack([ffc_block, oakd_row])


def parse_args():
    epilog = """
Examples:  %(prog)s --host localhost --sources both
           %(prog)s --rtsp-backend ffmpeg --host HOST
           %(prog)s --yolo --model weights.pt

Env: LUXONIS_SOURCES, LUXONIS_RTSP_HOST, LUXONIS_RTSP_BACKEND, OPENCV_FFMPEG_CAPTURE_OPTIONS
"""
    p = argparse.ArgumentParser(
        description="Tiled RTSP viewer (Luxonis encoder); optional YOLO.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    p.add_argument(
        "--sources",
        choices=("ffc", "oakd", "both"),
        default=os.environ.get("LUXONIS_SOURCES", "both"),
        help="Camera group(s) to display (must match running encoder).",
    )
    p.add_argument(
        "--host",
        default=os.environ.get("LUXONIS_RTSP_HOST", "localhost"),
        help="Host running RTSP server (default: localhost).",
    )
    p.add_argument(
        "--rtsp-backend",
        choices=("auto", "gstreamer", "ffmpeg"),
        default=os.environ.get("LUXONIS_RTSP_BACKEND", "auto"),
        help="RTSP decode: auto tries GStreamer then FFmpeg (pip opencv-python often needs ffmpeg).",
    )
    p.add_argument("--cell-width", type=int, default=480, help="Tile width in pixels.")
    p.add_argument("--cell-height", type=int, default=270, help="Tile height in pixels.")
    p.add_argument(
        "--rotate",
        choices=("none", "180"),
        default="none",
        help="Rotate each frame (e.g. upside-down cameras).",
    )
    p.add_argument(
        "--yolo",
        action="store_true",
        help="Run Ultralytics YOLO on each tile (see --model).",
    )
    p.add_argument(
        "--model",
        default="best_190_Epoch.pt",
        help="YOLO weights path (default: best_190_Epoch.pt).",
    )
    p.add_argument(
        "--conf",
        type=float,
        default=0.65,
        help="Minimum detection confidence for YOLO.",
    )
    p.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Run YOLO every Nth frame per stream (default: 1).",
    )
    p.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="YOLO inference size (default: 640).",
    )
    p.add_argument(
        "--half",
        action="store_true",
        help="Use FP16 YOLO inference if CUDA is available.",
    )
    p.add_argument(
        "--yolo-rgb-only",
        action="store_true",
        help="When --sources both, only run YOLO on oakd_rgb.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    names = streams_for_sources(args.sources)
    rotate_code = None
    if args.rotate == "180":
        rotate_code = cv2.ROTATE_180

    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        "rtsp_transport;tcp|buffer_size;1048576|max_delay;500000",
    )

    if not encoder_tcp_listening(args.host, 8554):
        print(
            f"\nNo RTSP server on {args.host}:8554 — start luxonis_encode_gstreamer.py "
            "(viewer retries until streams open).\n",
            flush=True,
        )

    readers: List[FrameReader] = []
    for n in names:
        url = rtsp_url(args.host, n)
        r = FrameReader(n, url, backend=args.rtsp_backend)
        r.start()
        readers.append(r)

    yolo: Optional[YoloRunner] = None
    if args.yolo:
        yolo = YoloRunner(
            args.model,
            args.conf,
            args.stride,
            args.imgsz,
            args.half,
            args.yolo_rgb_only,
        )

    show_labels = True
    yolo_enabled = args.yolo

    print("Controls: q=quit  l=toggle stream labels  y=toggle YOLO (if loaded)")
    print(f"Streaming {len(names)} camera(s) from rtsp://{args.host}:8554/ ...")

    window = "luxonis_viewer"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    try:
        while True:
            frames: Dict[str, np.ndarray] = {}
            for reader in readers:
                f = reader.get_frame()
                if f is not None:
                    if rotate_code is not None:
                        f = cv2.rotate(f, rotate_code)
                    frames[reader.name] = f

            cell_w, cell_h = args.cell_width, args.cell_height

            if yolo_enabled and yolo is not None:
                for name in list(frames.keys()):
                    frames[name] = yolo.annotate(name, frames[name])

            canvas = build_grid(args.sources, frames, cell_w, cell_h, show_labels)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("l"):
                show_labels = not show_labels
            if key == ord("y") and yolo is not None:
                yolo_enabled = not yolo_enabled
    finally:
        for r in readers:
            r.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
