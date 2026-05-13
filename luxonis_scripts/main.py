import argparse
import os
import subprocess
import threading
import time

import cv2
import depthai as dai
import numpy as np

from rtsp_server import RtspServer

FFC_MXID = "14442C10014791D700"
OAKD_MXID = "1944301001EDE12E00"
STREAM_NAMES = ["Front", "Right", "Left", "Back", "RGB", "DEPTH"]

MODES = {
    "ffc_all": {"bitrate": 2000000, "fps": 30},
    "ffc_front": {"bitrate": 7000000, "fps": 30},
    "ffc_back": {"bitrate": 7000000, "fps": 30},
    "ffc_right": {"bitrate": 7000000, "fps": 30},
    "ffc_left": {"bitrate": 7000000, "fps": 30},
    "oakd_rgb": {"bitrate": 7000000, "fps": 30},
    "oakd_yolo": {"bitrate": 4000000, "fps": 15},
}

ALL_MODES = list(MODES.keys())
MODEL_PATH = os.path.join(os.path.dirname(__file__), "best_190_Epoch.rvc2.tar.xz")


class StreamMetrics:
    def __init__(self):
        self._lock = threading.Lock()
        self._bytes = {}
        self._frames = {}
        self._last_sample = time.monotonic()

    def record(self, name, byte_count):
        with self._lock:
            self._bytes[name] = self._bytes.get(name, 0) + byte_count
            self._frames[name] = self._frames.get(name, 0) + 1

    def sample(self):
        with self._lock:
            now = time.monotonic()
            elapsed = max(now - self._last_sample, 1e-6)
            snap = {
                name: {
                    "mbps": (self._bytes[name] * 8 / elapsed) / 1e6,
                    "fps": self._frames[name] / elapsed,
                }
                for name in self._frames
            }
            self._bytes.clear()
            self._frames.clear()
            self._last_sample = now
            return snap, elapsed

    def reset(self):
        with self._lock:
            self._bytes.clear()
            self._frames.clear()
            self._last_sample = time.monotonic()


class PipelineSession:
    def __init__(self, server, metrics, visualizer):
        self._server = server
        self._metrics = metrics
        self._visualizer = visualizer
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._thread = None
        self._mode = None
        self._error = None
        self._detection_lock = threading.Lock()
        self._latest_detections = None
        self._rgb_lock = threading.Lock()
        self._latest_rgb = None

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

    def latest_detections(self):
        with self._detection_lock:
            return self._latest_detections

    def latest_rgb_frame(self):
        with self._rgb_lock:
            return None if self._latest_rgb is None else self._latest_rgb

    def _worker(self, mode, device_id):
        try:
            profile = profile_for(mode)
            fps = profile["fps"]
            bitrate = profile["bitrate"]

            device = dai.Device(dai.DeviceInfo(device_id))

            if mode in ("oakd_yolo", "oakd_depth", "oakd_all"):
                try:
                    device.setIrLaserDotProjectorIntensity(1.0)
                    device.setIrFloodLightIntensity(0.5)
                except Exception:
                    pass

            with dai.Pipeline(device) as pipeline:
                bitstream_queues, extra_queues = build_pipeline(pipeline, mode, fps, bitrate, self._visualizer)
                pipeline.start()
                self._visualizer.registerPipeline(pipeline)
                self._ready_event.set()

                detection_queue = extra_queues.get("detections")
                while pipeline.isRunning() and not self._stop_event.is_set():
                    for name, q in bitstream_queues.items():
                        if q.has():
                            data = q.get().getData()
                            self._server.factories[name].push(data)
                            self._metrics.record(name, len(data))
                    if detection_queue is not None and detection_queue.has():
                        with self._detection_lock:
                            self._latest_detections = detection_queue.get()
                    self._visualizer.waitKey(1)
        except Exception as e:
            self._error = str(e)
        finally:
            self._metrics.reset()
            self._ready_event.set()


def main():
    args = parse_args()

    server = RtspServer(stream_names=STREAM_NAMES)
    metrics = StreamMetrics()
    visualizer = dai.RemoteConnection(webSocketPort=8765, httpPort=8082)
    session = PipelineSession(server, metrics, visualizer)
    
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
            elif cmd == "bw":
                if not session.is_running():
                    print("Idle.")
                    continue
                print_bandwidth(metrics, session.current_mode())
            elif cmd == "watch":
                interval = float(parts[1]) if len(parts) >= 2 else 1.0
                watch_bandwidth(metrics, session, interval)
            elif cmd == "detections":
                detections = session.latest_detections()
                if detections is None:
                    print("No detections yet.")
                else:
                    for d in detections.detections:
                        print(f"  {d.labelName:<16} conf={d.confidence:.2f}  "
                            f"X={d.spatialCoordinates.x:>7.0f}mm  "
                            f"Y={d.spatialCoordinates.y:>7.0f}mm  "
                            f"Z={d.spatialCoordinates.z:>7.0f}mm")   
            elif cmd == "watch_detections":
                interval = float(parts[1]) if len(parts) >= 2 else 1.0
                try:
                    while session.is_running():
                        detections = session.latest_detections()
                        if detections is not None:
                            print(f"\n[{session.current_mode()}] Detections:")
                            for d in detections.detections:
                                print(f"  {d.labelName:<16} conf={d.confidence:.2f}  "
                                    f"X={d.spatialCoordinates.x:>7.0f}mm  "
                                    f"Y={d.spatialCoordinates.y:>7.0f}mm  "
                                    f"Z={d.spatialCoordinates.z:>7.0f}mm")   
                        else:
                            print(f"\n[{session.current_mode()}] No detections yet.")
                        time.sleep(interval)
                except KeyboardInterrupt:
                    print() # Create a new line after CTRL+C
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
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down.")
        session.stop()
        subprocess.Popen(["pkill", "-f", "run_cameras.sh"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def resolve_device(mode, override):
    if override:
        return override
    return FFC_MXID if mode.startswith("ffc_") else OAKD_MXID


def kit_name(mode):
    return "ffc" if mode.startswith("ffc_") else "oakd"


def print_help():
    print("Commands:")
    print("  mode <name> [device_id]  switch pipeline, device_id/IP is optional.  For the black mydlink router, 192.168.0.100, and 192.168.0.102 were usually either the OAK-D or FFC")
    print("  stop                     stop current pipeline")
    print("  status                   show current state")
    print("  bw                       bandwidth + fps per stream")
    print("  watch [interval]         monitor bandwidth at intervals")
    print("  detections               show latest detections (FFC YOLO or OAK-D YOLO modes only)")
    print("  watch_detections [interval]   monitor detections at intervals")
    print("  quit / exit / q          shut down")
    print(f"Modes: {', '.join(ALL_MODES)}")


## RUN bw to print the current bandwidth and fps
def print_bandwidth(metrics, mode):
    snap, elapsed = metrics.sample()
    if not snap:
        print(f"[{mode}] no traffic in last {elapsed:.2f}s")
        return
    configured = profile_for(mode)["bitrate"] / 1e6 if mode else 0.0
    total_mbps = sum(s["mbps"] for s in snap.values())
    print(f"[{mode}] window={elapsed:.2f}s  configured={configured:.1f} Mbps/stream  total={total_mbps:.2f} Mbps")
    print(f"  {'stream':<8} {'mbps':>7}  {'fps':>6}")
    for name in sorted(snap):
        s = snap[name]
        print(f"  {name:<8} {s['mbps']:>7.2f}  {s['fps']:>6.1f}")


## RUN watch [interval] to print bandwidth at set interval (in seconds)
def watch_bandwidth(metrics, session, interval):
    metrics.sample() 
    try:
        while session.is_running():
            time.sleep(interval)
            print_bandwidth(metrics, session.current_mode())
    except KeyboardInterrupt:
        print() # Create a new line after CTRL+C
    if not session.is_running():
        print("Idle.")


def parse_args():
    parser = argparse.ArgumentParser(description="Luxonis camera encoder script")
    parser.add_argument(
        "--mode",
        choices=ALL_MODES,
        default="ffc_all",
        help="Initial pipeline mode"
    )
    parser.add_argument(
        "-d", "--device",
        default=None,
        help="Default device MxId or IP"
    )
    return parser.parse_args()


def build_pipeline(pipeline, mode, maxFps, bitrate, visualizer):
    sockets = []
    if mode == "ffc_all" or mode == "ffc_yolo":
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

    bitstream_queues = {}
    extra_queues = {}

    for idx, (socket, name) in enumerate(sockets):
        cam = pipeline.create(dai.node.Camera).build(socket)
        cam_out = cam.requestOutput((1920, 1080), fps=maxFps, type=dai.ImgFrame.Type.NV12)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(maxFps, dai.VideoEncoderProperties.Profile.H264_MAIN)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrate(bitrate)
        enc.setKeyframeFrequency(maxFps)
        cam_out.link(enc.input)
        bitstream_queues[name] = enc.out.createOutputQueue(maxSize=30, blocking=True)

        mjpeg = pipeline.create(dai.node.VideoEncoder)
        mjpeg.setDefaultProfilePreset(maxFps, dai.VideoEncoderProperties.Profile.MJPEG)
        cam_out.link(mjpeg.input)
        visualizer.addTopic(name, mjpeg.out, "img")


    if mode == "oakd_yolo":
        cam_rgb   = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=maxFps)
        mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=maxFps)
        mono_right= pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=maxFps)
        
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setRectification(True)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True) #helps with close up objects

        mono_left.requestOutput((640, 400)).link(stereo.left)
        mono_right.requestOutput((640, 400)).link(stereo.right)

        archive = dai.NNArchive(MODEL_PATH) # Gets yolo model

        spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(cam_rgb, stereo, archive)
        spatial_nn.input.setBlocking(False)
        spatial_nn.setConfidenceThreshold(0.5)
        spatial_nn.setDepthLowerThreshold(200)  # sets cutoff distance to < 20 cm
        spatial_nn.setDepthUpperThreshold(8000) # sets cutoff distance > 8 m

        rgb_out = cam_rgb.requestOutput((1920, 1080), fps=maxFps, type=dai.ImgFrame.Type.NV12) # encode rgb cam output for rtsp server
        
        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(maxFps, dai.VideoEncoderProperties.Profile.H264_MAIN)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrate(bitrate)
        enc.setKeyframeFrequency(maxFps)

        rgb_out.link(enc.input)
        
        bitstream_queues = {"RGB": enc.out.createOutputQueue(maxSize=30, blocking=True)}

        extra_queues = {
            "detections": spatial_nn.out.createOutputQueue(maxSize=4, blocking=False),
        }

        visualizer.addTopic("RGB", spatial_nn.passthrough, "img")
        visualizer.addTopic("Depth", spatial_nn.passthroughDepth, "img")
        visualizer.addTopic("Detections", spatial_nn.out, "img")

    return bitstream_queues, extra_queues
    
 
### Helper Functions ####

def profile_for(mode):
    if mode not in MODES:
        raise ValueError(f"No profile for mode '{mode}'")
    return MODES[mode]


if __name__ == "__main__":
    main()