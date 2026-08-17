# ruff: noqa: D100, D101, D102, D107, INP001
import contextlib
import queue
import threading
from collections.abc import Iterable

import gi
from gi.repository.Gst import Buffer, Element
from gi.repository.GstRtsp import RTSPUrl
from gi.repository.GstRtspServer import RTSPMedia

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402


# --- RTSP: one factory per stream (H.264 byte stream in, RTP out) ---
class RtspFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()

        self.set_shared(True)
        self.queue = queue.Queue(maxsize=2)

        self.launch_string = (
            "appsrc name=source is-live=true block=true format=GST_FORMAT_TIME "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au "
            "! h264parse "
            "! rtph264pay name=pay0 pt=96 mtu=1200 config-interval=1"
        )

    def push(self, data: bytes):
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self.queue.get_nowait()
            self.queue.put_nowait(data)

    def do_create_element(self, url: RTSPUrl) -> Element:  # noqa: ARG002
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, media: RTSPMedia):
        appsrc = media.get_element().get_child_by_name("source")  # ty:ignore[unresolved-attribute]

        appsrc.set_property("format", Gst.Format.TIME)
        appsrc.set_property("do-timestamp", True)

        media.set_reusable(True)

        appsrc.connect("need-data", self.on_need_data)

    def on_need_data(self, src: Element, _length: int):
        try:
            data = self.queue.get(timeout=1)
            buf: Buffer = Gst.Buffer.new_allocate(None, len(data), None)  # ty:ignore[invalid-assignment]
            buf.fill(0, data)  # ty:ignore[unresolved-attribute]
            src.emit("push-buffer", buf)
        except queue.Empty:
            pass


# --- RTSP server ---
class RtspServer(GstRtspServer.RTSPServer):
    def __init__(self, stream_names: Iterable[str]):
        super().__init__()
        Gst.init(None)

        self.factories = {}
        mounts = self.get_mount_points()
        if mounts is None:
            raise RuntimeError("mounts must not be None")

        for name in stream_names:
            factory = RtspFactory()
            mounts.add_factory(f"/{name}", factory)
            self.factories[name] = factory

        self.attach(None)

        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
