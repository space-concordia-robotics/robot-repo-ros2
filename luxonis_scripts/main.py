import argparse
import time
import depthai as dai

from rtsp_server import RtspServer

FFC_MXID = "14442C10014791D700"
OAKD_MXID = "1944301001EDE12E00"

class PipelineType:
    MODE_FFC_ALL = {"name": "ffc_all", "bitrate": 2000000, "fps": 30}
    MODE_FFC_FRONT = {"name": "ffc_front", "bitrate": 7000000, "fps": 30}
    MODE_FFC_BACK = {"name": "ffc_back", "bitrate": 7000000, "fps": 30}
    MODE_FFC_RIGHT = {"name": "ffc_right", "bitrate": 7000000, "fps": 30}
    MODE_FFC_LEFT = {"name": "ffc_left", "bitrate": 7000000, "fps": 30}
    MODE_OAK_ALL = {"name": "oakd_all", "bitrate": 1000000, "fps": 5}
    MODE_OAKD_RGB = {"name": "oakd_rgb", "bitrate": 7000000, "fps": 30}
    MODE_OAKD_DEPTH = {"name": "oakd_depth", "bitrate": 1000000, "fps": 5}

def main():
    args = parse_args()
    mode = args.mode

    profile = profile_for(mode)
    fps = profile["fps"]
    bitrate = profile["bitrate"]

    using_ffc = mode in ["ffc_all", "ffc_front", "ffc_back", "ffc_right", "ffc_left"]
    device_name = "ffc" if using_ffc else "oakd"
    target_id = args.device or (FFC_MXID if using_ffc else OAKD_MXID)

    print(f"Connecting to {device_name.upper()} device ({target_id})...")

    try:
        device = dai.Device(dai.DeviceInfo(target_id))
        with dai.Pipeline(device) as pipeline:
            stream_names, bitstream_queues, extra_queues = build_pipeline(pipeline, mode, fps, bitrate)
            server = RtspServer(stream_names=stream_names)

            pipeline.start()
            print("Pipeline started. Streaming...")
            try:
                while pipeline.isRunning():
                    for stream_name, q in zip(stream_names, bitstream_queues):
                        if q.has():
                            data = q.get().getData()
                            server.factories[stream_name].push(data)
                    time.sleep(0.001)
            except KeyboardInterrupt:
                print("\nShutting down.")
    except RuntimeError as e:
        print(f"Failed to connect to {device_name.upper()}: {e}")

    

def parse_args():
    parser = argparse.ArgumentParser(description="Luxonis camera encoder script")
    parser.add_argument(
        "--mode",
        choices=["ffc_all", "ffc_front", "ffc_back", "ffc_right", "ffc_left", "oakd_all", "oakd_rgb", "oakd_depth"],
        default="ffc_all",
        help="Which pipeline mode to start with"
    )
    parser.add_argument(
        "-d", "--device",
        default=None,
        help="Device MxId or IP (overrides FFC/OAKD MXID constants; useful for PoE)"
    )
    return parser.parse_args()


def build_pipeline(pipeline, mode, maxFps, bitrate):
    if mode == "ffc_all":
        sockets = [
            dai.CameraBoardSocket.CAM_A,
            dai.CameraBoardSocket.CAM_B,
            dai.CameraBoardSocket.CAM_C,
            dai.CameraBoardSocket.CAM_D,
        ]
        stream_names = ["FRONT", "RIGHT", "LEFT", "BACK"]
    elif mode == "ffc_front":
        sockets = [dai.CameraBoardSocket.CAM_A]
        stream_names = ["FRONT"]
    elif mode == "ffc_back":
        sockets = [dai.CameraBoardSocket.CAM_D]
        stream_names = ["BACK"]
    elif mode == "ffc_right":
        sockets = [dai.CameraBoardSocket.CAM_B]
        stream_names = ["RIGHT"]
    elif mode == "ffc_left":
        sockets = [dai.CameraBoardSocket.CAM_C]
        stream_names = ["LEFT"]
    elif mode == "oakd_rgb":
        sockets = [dai.CameraBoardSocket.CAM_A]
        stream_names = ["RGB"]
    elif mode == "oakd_depth":
        sockets = [
                dai.CameraBoardSocket.CAM_B,
                dai.CameraBoardSocket.CAM_C
                    ]
        stream_names = ["DEPTH"]
    elif mode == "oakd_all":
        sockets = [dai.CameraBoardSocket.CAM_A]
        stream_names = ["RGB"]

    bitstream_queues = []
    for socket in sockets:
        cam = pipeline.create(dai.node.Camera).build(socket)
        cam_out = cam.requestOutput((1920, 1080), fps=maxFps, type=dai.ImgFrame.Type.NV12)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(maxFps, dai.VideoEncoderProperties.Profile.H264_MAIN)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrate(bitrate)
        enc.setKeyframeFrequency(maxFps)

        cam_out.link(enc.input)
        bitstream_queues.append(enc.out.createOutputQueue(maxSize=30, blocking=True))

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

    return stream_names, bitstream_queues, extra_queues
    
 
### Helper Functions ####

def profile_for(mode):
    for value in vars(PipelineType).values():
        if isinstance(value, dict) and value.get("name") == mode:
            return value
    raise ValueError(f"No PipelineType profile for mode '{mode}'")


if __name__ == "__main__":
    main()