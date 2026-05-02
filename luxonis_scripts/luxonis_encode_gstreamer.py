#!/usr/bin/env python3
"""DepthAI → H.265 → RTSP (port 8554). FFC 4P and/or OAK-D; USB or Ethernet devices.

Deps: see system_deps_encoder.txt (apt `gi` + pip `depthai`, often venv with system site-packages).
Viewer: luxonis_viewer.py (start this script first).
"""

import argparse
import json
import os
import threading
import queue
import time

try:
    import gi
except ModuleNotFoundError as e:
    raise SystemExit(
        "Missing Python module `gi` (install with apt, not pip `gi`).\n"
        "  sudo apt install python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-rtsp-server-1.0\n"
        "Details: luxonis_scripts/system_deps_encoder.txt"
    ) from e

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import Gst, GstRtspServer, GLib

try:
    import depthai as dai
except ModuleNotFoundError as e:
    raise SystemExit(
        "Missing `depthai` (pip install depthai in the same Python you use here).\n"
        "For apt `gi` + pip packages, use a venv with system site-packages — see\n"
        "luxonis_scripts/system_deps_encoder.txt"
    ) from e

FFC_STREAMS = ["ffc0", "ffc1", "ffc2", "ffc3"]
OAKD_STREAMS = ["oakd_rgb", "oakd_left", "oakd_right"]

DEFAULT_FFC_MXID = "14442C10014791D700"
DEFAULT_OAKD_MXID = "1944301001EDE12E00"


def load_config(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def resolve_mx_ids(args):
    ffc = DEFAULT_FFC_MXID
    oakd = DEFAULT_OAKD_MXID
    if args.config:
        cfg = load_config(args.config)
        ffc = cfg.get("ffc_mxid", ffc)
        oakd = cfg.get("oakd_mxid", oakd)
    ffc = os.environ.get("LUXONIS_FFC_MXID", ffc)
    oakd = os.environ.get("LUXONIS_OAKD_MXID", oakd)
    if args.ffc_mxid:
        ffc = args.ffc_mxid
    if args.oakd_mxid:
        oakd = args.oakd_mxid
    return ffc, oakd


def expected_device_count(sources):
    if sources == "both":
        return 2
    return 1


# --- RTSP: one factory per stream (H.265 byte stream in, RTP out) ---
class RtspFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()

        self.set_shared(True)
        self.queue = queue.Queue(maxsize=2)

        self.launch_string = (
            "appsrc name=source is-live=true block=true format=GST_FORMAT_TIME "
            "caps=video/x-h265,stream-format=byte-stream,alignment=au "
            "! h265parse "
            "! rtph265pay name=pay0 pt=96 mtu=1200 config-interval=1"
        )

    def push(self, data: bytes):
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            pass  # drop if client is too slow

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        appsrc = rtsp_media.get_element().get_child_by_name("source")

        # RTSP live mode (avoids 503 with many clients)
        appsrc.set_property("is-live", True)
        appsrc.set_property("format", Gst.Format.TIME)
        appsrc.set_property("do-timestamp", True)

        rtsp_media.set_reusable(True)

        appsrc.connect("need-data", self.on_need_data)

    def on_need_data(self, src, length):
        try:
            data = self.queue.get(timeout=1)
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            src.emit("push-buffer", buf)
        except queue.Empty:
            pass


# --- RTSP server ---
class RtspServer(GstRtspServer.RTSPServer):
    def __init__(self, stream_names):
        super().__init__()
        Gst.init(None)

        self.factories = {}
        mounts = self.get_mount_points()

        for name in stream_names:
            factory = RtspFactory()
            mounts.add_factory(f"/{name}", factory)
            self.factories[name] = factory

        self.attach(None)

        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()


# --- Per-device capture threads ---
def run_ffc_device(server, device_info, fps, bitrate):
    while True:
        try:
            pipeline = dai.Pipeline()

            sockets = [
                dai.CameraBoardSocket.CAM_A,
                dai.CameraBoardSocket.CAM_B,
                dai.CameraBoardSocket.CAM_C,
                dai.CameraBoardSocket.CAM_D,
            ]
            stream_names = ["ffc0", "ffc1", "ffc2", "ffc3"]

            for i, sock in enumerate(sockets):
                cam = pipeline.create(dai.node.ColorCamera)
                cam.setBoardSocket(sock)
                cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                cam.setVideoSize(1920, 1080)
                cam.setFps(fps)
                cam.setInterleaved(False)

                enc = pipeline.create(dai.node.VideoEncoder)
                enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
                enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
                enc.setBitrate(bitrate)
                enc.setKeyframeFrequency(fps)

                cam.video.link(enc.input)

                xout = pipeline.create(dai.node.XLinkOut)
                xout.setStreamName(stream_names[i])
                enc.bitstream.link(xout.input)

            with dai.Device(pipeline, device_info) as device:
                queues = [device.getOutputQueue(n, maxSize=30, blocking=True) for n in stream_names]
                print(f"  FFC 4P ({device_info.getMxId()}) running")
                time.sleep(2)
                while True:
                    for name, q in zip(stream_names, queues):
                        pkt = q.tryGet()
                        if pkt is not None:
                            server.factories[name].push(pkt.getData())
        except Exception as e:
            print(f"  FFC 4P error: {e} — reconnecting in 1s...")
            time.sleep(1)


def run_oakd_device(server, device_info, fps, bitrate):
    while True:
        try:
            pipeline = dai.Pipeline()

            configs = [
                ("rgb", dai.CameraBoardSocket.CAM_A, dai.ColorCameraProperties.SensorResolution.THE_1080_P),
                ("left", dai.CameraBoardSocket.CAM_B, dai.ColorCameraProperties.SensorResolution.THE_800_P),
                ("right", dai.CameraBoardSocket.CAM_C, dai.ColorCameraProperties.SensorResolution.THE_800_P),
            ]
            stream_names = ["oakd_rgb", "oakd_left", "oakd_right"]

            for i, (label, sock, res) in enumerate(configs):
                cam = pipeline.create(dai.node.ColorCamera)
                cam.setBoardSocket(sock)
                cam.setResolution(res)
                cam.setVideoSize(1280, 800)
                cam.setFps(fps)
                cam.setInterleaved(False)

                enc = pipeline.create(dai.node.VideoEncoder)
                enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
                enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
                enc.setBitrate(bitrate)
                enc.setKeyframeFrequency(fps)

                cam.video.link(enc.input)

                xout = pipeline.create(dai.node.XLinkOut)
                xout.setStreamName(stream_names[i])
                enc.bitstream.link(xout.input)

            with dai.Device(pipeline, device_info) as device:
                queues = [device.getOutputQueue(n, maxSize=30, blocking=True) for n in stream_names]
                print(f"  OAK-D Pro W ({device_info.getMxId()}) running")
                time.sleep(2)
                while True:
                    for name, q in zip(stream_names, queues):
                        pkt = q.tryGet()
                        if pkt is not None:
                            server.factories[name].push(pkt.getData())
        except Exception as e:
            print(f"  OAK-D Pro W error: {e} — reconnecting in 1s...")
            time.sleep(1)


# --- Discovery ---
def discover_devices(expected=2, max_attempts=10, delay=1):
    devices = []
    for attempt in range(1, max_attempts + 1):
        devices = dai.Device.getAllAvailableDevices()
        print(f"  Attempt {attempt}/{max_attempts}: found {len(devices)} device(s)")
        for d in devices:
            print(f"    MxId: {d.getMxId()}  State: {d.state.name}")

        if len(devices) >= expected:
            return devices

        if attempt < max_attempts:
            print(f"  Waiting {delay}s before retrying...")
            time.sleep(delay)

    print(f"\n  Could not find {expected} device(s) after {max_attempts} attempts.")
    print(f"  Proceeding with {len(devices)} device(s).\n")
    return devices


def identify_devices(devices, ffc_mxid, oakd_mxid):
    ffc_info = None
    oakd_info = None

    for d in devices:
        if d.getMxId() == ffc_mxid:
            ffc_info = d
        elif d.getMxId() == oakd_mxid:
            oakd_info = d

    return ffc_info, oakd_info


def parse_args():
    epilog = """
Start this before luxonis_viewer.py. RTSP: rtsp://HOST:8554/<stream> (default port 8554).

Install: luxonis_scripts/system_deps_encoder.txt
Env: LUXONIS_SOURCES, LUXONIS_FFC_MXID, LUXONIS_OAKD_MXID
Config: --config JSON with ffc_mxid, oakd_mxid (see luxonis_devices.example.json)
"""
    p = argparse.ArgumentParser(
        description="Luxonis FFC 4P and/or OAK-D → H.265 RTSP.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    p.add_argument(
        "--sources",
        choices=("ffc", "oakd", "both"),
        default=os.environ.get("LUXONIS_SOURCES", "both"),
        help="Which camera groups to expose (default: both or LUXONIS_SOURCES).",
    )
    p.add_argument(
        "--config",
        metavar="PATH",
        help="JSON file with optional keys ffc_mxid, oakd_mxid.",
    )
    p.add_argument("--ffc-mxid", metavar="ID", help="MxId of the FFC 4P device (overrides config/env).")
    p.add_argument("--oakd-mxid", metavar="ID", help="MxId of the OAK-D device (overrides config/env).")
    p.add_argument("--fps", type=int, default=30, help="Video FPS (default: 30).")
    p.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="H.265 bits/s per stream (default 1e6). Lower if many streams on a slow link.",
    )
    p.add_argument(
        "--discovery-attempts",
        type=int,
        default=10,
        help="Max discovery attempts (default: 10).",
    )
    p.add_argument(
        "--discovery-delay",
        type=float,
        default=1.0,
        help="Seconds between discovery attempts (default: 5).",
    )
    return p.parse_args()


def main():
    args = parse_args()
    fps = args.fps
    bitrate = args.bitrate
    sources = args.sources

    ffc_mxid, oakd_mxid = resolve_mx_ids(args)

    want_ffc = sources in ("ffc", "both")
    want_oakd = sources in ("oakd", "both")
    expected = expected_device_count(sources)

    print("\nSearching for DepthAI devices...")
    print(
        f"  Mode: --sources {sources} (expecting at least {expected} device(s) on USB/network for full setup)"
    )
    devices = discover_devices(
        expected=expected, max_attempts=args.discovery_attempts, delay=args.discovery_delay
    )

    ffc_info, oakd_info = identify_devices(devices, ffc_mxid, oakd_mxid)

    if want_ffc:
        if ffc_info:
            print(f"\n  FFC 4P      -> {ffc_info.getMxId()}")
        else:
            print("\n  FFC 4P      -> NOT FOUND (FFC streams will not be available)")
    if want_oakd:
        if oakd_info:
            print(f"  OAK-D Pro W -> {oakd_info.getMxId()}")
        else:
            print("  OAK-D Pro W -> NOT FOUND (OAK-D streams will not be available)")

    if sources == "both" and (ffc_info is None or oakd_info is None):
        print(
            "\n  Note: --sources both but one device type was not found; "
            "RTSP will only expose streams for connected hardware."
        )

    stream_names = []
    if want_ffc and ffc_info:
        stream_names.extend(FFC_STREAMS)
    if want_oakd and oakd_info:
        stream_names.extend(OAKD_STREAMS)

    if not stream_names:
        print("\nNo RTSP streams to expose: no matching devices for --sources %s. Exiting." % sources)
        return

    server = RtspServer(stream_names)

    if want_ffc and ffc_info:
        t = threading.Thread(target=run_ffc_device, args=(server, ffc_info, fps, bitrate), daemon=True)
        t.start()
    elif want_ffc and not ffc_info:
        print("  Warning: FFC requested but device not found; not starting FFC pipeline.")

    if want_oakd and oakd_info:
        t = threading.Thread(target=run_oakd_device, args=(server, oakd_info, fps, bitrate), daemon=True)
        t.start()
    elif want_oakd and not oakd_info:
        print("  Warning: OAK-D requested but device not found; not starting OAK-D pipeline.")

    print("\nRTSP streams ready:")
    for name in stream_names:
        print(f"  rtsp://localhost:8554/{name}")
    print("\nWaiting for data...\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down.")


if __name__ == "__main__":
    main()
