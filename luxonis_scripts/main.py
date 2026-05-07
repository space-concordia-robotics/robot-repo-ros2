import argparse
import threading
import time
import depthai as dai

from rtsp_server import RtspServer

FFC_MXID = "14442C10014791D700"
OAKD_MXID = "1944301001EDE12E00"
STREAM_NAMES = ["FRONT", "RIGHT", "LEFT", "BACK", "RGB", "DEPTH"]
ALL_MODES = [
    "ffc_all", "ffc_front", "ffc_back", "ffc_right", "ffc_left",
    "oakd_all", "oakd_rgb", "oakd_depth",
]

class PipelineType:
    MODE_FFC_ALL = {"name": "ffc_all", "bitrate": 2000000, "fps": 30}
    MODE_FFC_FRONT = {"name": "ffc_front", "bitrate": 7000000, "fps": 30}
    MODE_FFC_BACK = {"name": "ffc_back", "bitrate": 7000000, "fps": 30}
    MODE_FFC_RIGHT = {"name": "ffc_right", "bitrate": 7000000, "fps": 30}
    MODE_FFC_LEFT = {"name": "ffc_left", "bitrate": 7000000, "fps": 30}
    MODE_OAK_ALL = {"name": "oakd_all", "bitrate": 1000000, "fps": 5}
    MODE_OAKD_RGB = {"name": "oakd_rgb", "bitrate": 7000000, "fps": 30}
    MODE_OAKD_DEPTH = {"name": "oakd_depth", "bitrate": 1000000, "fps": 5}


class PipelineSession:
    def __init__(self, server):
        self._server = server
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._thread = None
        self._mode = None
        self._error = None

    def start(self, mode, device_id):
        with self._lock:
            self._stop_locked()
            self._stop_event.clear()
            self._ready_event.clear()
            self._error = None
            self._mode = mode
            self._thread = threading.Thread(
                target=self._worker,
                args=(mode, device_id),
                daemon=False,
            )
            self._thread.start()

        self._ready_event.wait()
        if self._error is not None:
            with self._lock:
                self._thread = None
                self._mode = None
            raise RuntimeError(self._error)

    def stop(self):
        with self._lock:
            self._stop_locked()

    def is_running(self):
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def current_mode(self):
        return self._mode if self.is_running() else None

    def _stop_locked(self):
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=10)
        self._thread = None
        self._mode = None

    def _worker(self, mode, device_id):
        try:
            profile = profile_for(mode)
            fps = profile["fps"]
            bitrate = profile["bitrate"]

            device = dai.Device(dai.DeviceInfo(device_id))
            with dai.Pipeline(device) as pipeline:
                bitstream_queues, _ = build_pipeline(pipeline, mode, fps, bitrate)
                pipeline.start()
                self._ready_event.set()

                while pipeline.isRunning() and not self._stop_event.is_set():
                    for name, q in bitstream_queues.items():
                        if q.has():
                            data = q.get().getData()
                            self._server.factories[name].push(data)
                    time.sleep(0.001)
        except Exception as e:
            self._error = str(e)
        finally:
            self._ready_event.set()


def main():
    args = parse_args()

    server = RtspServer(stream_names=STREAM_NAMES)
    session = PipelineSession(server)

    if args.mode:
        device_id = resolve_device(args.mode, args.device)
        print(f"Connecting to {kit_name(args.mode).upper()} ({device_id})...")
        try:
            session.start(args.mode, device_id)
            print(f"Started: {args.mode}")
        except RuntimeError as e:
            print(f"Failed: {e}")

    print_help()

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                break
            elif cmd in ("help", "?"):
                print_help()
            elif cmd == "stop":
                session.stop()
                print("Stopped.")
            elif cmd == "status":
                if session.is_running():
                    print(f"Running: {session.current_mode()}")
                else:
                    print("Idle.")
            elif cmd == "mode":
                if len(parts) < 2:
                    print("Usage: mode <name> [device_id]")
                    continue
                mode = parts[1]
                if mode not in ALL_MODES:
                    print(f"Unknown mode: {mode}")
                    continue
                device_id = parts[2] if len(parts) >= 3 else resolve_device(mode, args.device)
                print(f"Connecting to {kit_name(mode).upper()} ({device_id})...")
                try:
                    session.start(mode, device_id)
                    print(f"Started: {mode}")
                except RuntimeError as e:
                    print(f"Failed: {e}")
            else:
                print(f"Unknown command: {cmd} (type ? for help)")
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down.")
        session.stop()


def resolve_device(mode, override):
    if override:
        return override
    return FFC_MXID if mode.startswith("ffc_") else OAKD_MXID


def kit_name(mode):
    return "ffc" if mode.startswith("ffc_") else "oakd"


def print_help():
    print("Commands:")
    print("  mode <name> [device_id]  switch pipeline (omit device_id to use kit MXID or --device)")
    print("  stop                     stop current pipeline")
    print("  status                   show current state")
    print("  help / ?                 show this")
    print("  quit / exit / q          shut down")
    print(f"Modes: {', '.join(ALL_MODES)}")


def parse_args():
    parser = argparse.ArgumentParser(description="Luxonis camera encoder script")
    parser.add_argument(
        "--mode",
        choices=ALL_MODES,
        default="ffc_all",
        help="Initial pipeline mode (can be changed at runtime via the mode command)"
    )
    parser.add_argument(
        "-d", "--device",
        default=None,
        help="Default device MxId or IP override (useful for PoE; can be overridden per-mode)"
    )
    return parser.parse_args()


def build_pipeline(pipeline, mode, maxFps, bitrate):
    if mode == "ffc_all":
        sockets = [
            (dai.CameraBoardSocket.CAM_A, "FRONT"),
            (dai.CameraBoardSocket.CAM_B, "RIGHT"),
            (dai.CameraBoardSocket.CAM_C, "LEFT"),
            (dai.CameraBoardSocket.CAM_D, "BACK"),
        ]
    elif mode == "ffc_front":
        sockets = [(dai.CameraBoardSocket.CAM_A, "FRONT")]
    elif mode == "ffc_back":
        sockets = [(dai.CameraBoardSocket.CAM_D, "BACK")]
    elif mode == "ffc_right":
        sockets = [(dai.CameraBoardSocket.CAM_B, "RIGHT")]
    elif mode == "ffc_left":
        sockets = [(dai.CameraBoardSocket.CAM_C, "LEFT")]
    elif mode == "oakd_rgb":
        sockets = [(dai.CameraBoardSocket.CAM_A, "RGB")]
    elif mode == "oakd_depth":
        sockets = [(dai.CameraBoardSocket.CAM_B, "DEPTH")]
    elif mode == "oakd_all":
        sockets = [(dai.CameraBoardSocket.CAM_A, "RGB")]

    bitstream_queues = {}
    for socket, name in sockets:
        cam = pipeline.create(dai.node.Camera).build(socket)
        cam_out = cam.requestOutput((1920, 1080), fps=maxFps, type=dai.ImgFrame.Type.NV12)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(maxFps, dai.VideoEncoderProperties.Profile.H264_MAIN)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrate(bitrate)
        enc.setKeyframeFrequency(maxFps)

        cam_out.link(enc.input)
        bitstream_queues[name] = enc.out.createOutputQueue(maxSize=30, blocking=True)

    extra_queues = {}
    if mode == "oakd_all":
        left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setRectification(True)
        stereo.setLeftRightCheck(True)

        left_cam.requestFullResolutionOutput().link(stereo.left)
        right_cam.requestFullResolutionOutput().link(stereo.right)

        extra_queues["depth"] = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    return bitstream_queues, extra_queues
    
 
### Helper Functions ####

def profile_for(mode):
    for value in vars(PipelineType).values():
        if isinstance(value, dict) and value.get("name") == mode:
            return value
    raise ValueError(f"No PipelineType profile for mode '{mode}'")


if __name__ == "__main__":
    main()