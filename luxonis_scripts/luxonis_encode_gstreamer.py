#!/usr/bin/env python3

import threading
import queue
import time
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import Gst, GstRtspServer, GLib
import depthai as dai


# -------------------------------------------------
# RTSP MEDIA FACTORY (ONE PER CAMERA)
# -------------------------------------------------
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

        # CRITICAL: these prevent 503 errors
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


# -------------------------------------------------
# RTSP SERVER
# -------------------------------------------------
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


# -------------------------------------------------
# DEVICE RUNNER THREADS
# -------------------------------------------------
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
                cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
                cam.setVideoSize(640, 360)
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
                ("rgb",   dai.CameraBoardSocket.CAM_A, dai.ColorCameraProperties.SensorResolution.THE_1080_P),
                ("left",  dai.CameraBoardSocket.CAM_B, dai.ColorCameraProperties.SensorResolution.THE_800_P),
                ("right", dai.CameraBoardSocket.CAM_C, dai.ColorCameraProperties.SensorResolution.THE_800_P),
            ]
            stream_names = ["oakd_rgb", "oakd_left", "oakd_right"]

            for i, (label, sock, res) in enumerate(configs):
                cam = pipeline.create(dai.node.ColorCamera)
                cam.setBoardSocket(sock)
                cam.setResolution(res)
                cam.setVideoSize(640, 360)
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


# -------------------------------------------------
# DEVICE DISCOVERY
# -------------------------------------------------
def discover_devices(expected=2, max_attempts=10, delay=1):
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

    print(f"\n  Could not find {expected} devices after {max_attempts} attempts.")
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


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main():
    FPS = 30
    BITRATE = 1_000_000

    #   FFC 4P      -> 14442C10014791D700 (4 cameras)
    #   OAK-D Pro W -> 1944301001EDE12E00 (3 cameras)
    FFC_MXID = "14442C10014791D700"
    OAKD_MXID = "1944301001EDE12E00"

    # Attempt to discover devices, tries 10 times with 5s delay between
    print("\nSearching for DepthAI devices...")
    devices = discover_devices(expected=2, max_attempts=10, delay=5)

    # Identify which device is which
    ffc_info, oakd_info = identify_devices(devices, FFC_MXID, OAKD_MXID)

    if ffc_info:
        print(f"\n  FFC 4P      -> {ffc_info.getMxId()}")
    else:
        print("\n  FFC 4P      -> NOT FOUND")
    if oakd_info:
        print(f"  OAK-D Pro W -> {oakd_info.getMxId()}")
    else:
        print("  OAK-D Pro W -> NOT FOUND")

    # Stream names for the RTSP server
    all_streams = ["ffc0", "ffc1", "ffc2", "ffc3", "oakd_rgb", "oakd_left", "oakd_right"]

    server = RtspServer(all_streams)

    threads = []

    if ffc_info:
        t = threading.Thread(target=run_ffc_device, args=(server, ffc_info, FPS, BITRATE), daemon=True)
        t.start()
        threads.append(t)

    if oakd_info:
        t = threading.Thread(target=run_oakd_device, args=(server, oakd_info, FPS, BITRATE), daemon=True)
        t.start()
        threads.append(t)

    print("\nRTSP streams ready:")
    for name in all_streams:
        print(f"  rtsp://localhost:8554/{name}")
    print("\nWaiting for data...\n")

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down.")


if __name__ == "__main__":
    main()

