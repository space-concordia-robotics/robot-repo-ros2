#!/usr/bin/env python4

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
    def __init__(self, num_cams):
        super().__init__()
        Gst.init(None)

        self.factories = {}
        mounts = self.get_mount_points()

        for i in range(num_cams):
            factory = RtspFactory()
            mounts.add_factory(f"/cam{i}", factory)
            self.factories[i] = factory

        self.attach(None)

        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main():
    NUM_CAMS = 4
    FPS = 30
    BITRATE = 1_500_000

    # ---------- RTSP ----------
    server = RtspServer(NUM_CAMS)

    # ---------- DEPTHAI PIPELINE ----------
    pipeline = dai.Pipeline()

    sockets = [
        dai.CameraBoardSocket.CAM_A,
        dai.CameraBoardSocket.CAM_B,
        dai.CameraBoardSocket.CAM_C,
        dai.CameraBoardSocket.CAM_D,
    ]

    for i, sock in enumerate(sockets):
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(sock)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setVideoSize(640,360)
        cam.setFps(FPS)
        cam.setInterleaved(False)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H265_MAIN)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrate(BITRATE)
        enc.setKeyframeFrequency(FPS)  # stabilize RTSP

        cam.video.link(enc.input)
        
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName(f"AVC{i}")
        enc.bitstream.link(xout.input)

    # ---------- DEVICE ----------
    with dai.Device(pipeline) as device:
        queues = [
            device.getOutputQueue(f"AVC{i}", maxSize=30, blocking=True)
            for i in range(NUM_CAMS)
        ]
        print("\nRTSP streams ready:")
        for i in range(NUM_CAMS):
            print(f"  rtsp://localhost:8554/cam{i}")
        print("\nWaiting for data...\n")

        # Give cameras time to start producing frames (prevents 503)
        time.sleep(2)

        while True:
            for i, q in enumerate(queues):
                pkt = q.tryGet()
                if pkt is not None:
                    server.factories[i].push(pkt.getData())

if __name__ == "__main__":
    main()

