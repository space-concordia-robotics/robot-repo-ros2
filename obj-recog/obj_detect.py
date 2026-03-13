import cv2
from ultralytics import YOLO
import os
import glob
import sys
from tqdm import tqdm

# CONFIG
MODEL_PATH = "best_190_Epoch.pt"

INPUT_FOLDER = "" # Input Directory for folder 
OUTPUT_FOLDER = "" # Output Directory for folder 

MIN_CONF = 0.65 # Change according to video quality 
ARUCO_TYPE = "DICT_4X4_50" # Only effects Aruco Detection

# LOAD MODEL
try:
    model = YOLO(MODEL_PATH)
    print("YOLO model loaded successfully.")
except Exception as e:
    print("Failed to load YOLO model:", e)
    sys.exit()

# CREATE OUTPUT DIRECTORY
os.makedirs(OUTPUT_FOLDER, exist_ok=True)


# ARUCO SETUP

try:
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_TYPE))
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    print("ArUco detector initialized.")
except Exception as e:
    print("Failed to initialize ArUco:", e)
    sys.exit()


# GET VIDEO FILES

video_files = []
video_files.extend(glob.glob(os.path.join(INPUT_FOLDER, "*.mp4")))
video_files.extend(glob.glob(os.path.join(INPUT_FOLDER, "*.mov")))
video_files.extend(glob.glob(os.path.join(INPUT_FOLDER, "*.avi")))

if len(video_files) == 0:
    print("No video files found in folder.")
    sys.exit()

print(f"Found {len(video_files)} video(s).")

# VIDEO PROCESS FUNCTION

def process_video(video_path):

    print(f"\nProcessing: {os.path.basename(video_path)}")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error opening:", video_path)
        return

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        fps = 30

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    filename = os.path.basename(video_path)
    output_path = os.path.join(OUTPUT_FOLDER, f"processed_{filename}")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    # Progress bar
    with tqdm(total=total_frames,
              desc=filename,
              unit="frame",
              ncols=100) as pbar:

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # YOLO INFERENCE
            try:
                results = model(frame, verbose=False)

                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf < MIN_CONF:
                            continue

                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls = int(box.cls[0])
                        label = model.names[cls]

                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"{label}",
                            (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2
                        )

            except Exception as e:
                print("YOLO error:", e)

            # ARUCO DETECTION
            try:
                corners, ids, _ = aruco_detector.detectMarkers(frame)

                if ids is not None:
                    ids = ids.flatten()

                    for marker_corners, marker_id in zip(corners, ids):
                        pts = marker_corners[0].astype(int)
                        cv2.polylines(frame, [pts], True, (255, 0, 0), 2)

                        center_x = int(pts[:, 0].mean())
                        center_y = int(pts[:, 1].mean())

                        cv2.putText(
                            frame,
                            f"ID: {marker_id} | {ARUCO_TYPE}",
                            (center_x - 60, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 0, 0),
                            2
                        )

            except Exception as e:
                print("ArUco error:", e)

            out.write(frame)
            pbar.update(1)

    cap.release()
    out.release()

    print(f"Saved: {output_path}")

# MAIN LOOP
for video_path in video_files:
    process_video(video_path)

cv2.destroyAllWindows()
print("\nAll videos processed successfully.")
