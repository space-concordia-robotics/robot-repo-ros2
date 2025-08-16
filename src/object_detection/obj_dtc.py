import ultralytics
import supervision
import torch
import cv2
from collections import defaultdict
import supervision as sv
from ultralytics import YOLO
# imported lib for obj detection

# Load a pretrained YOLOv8n model
model = YOLO('yolov8n.pt')

# Run inference on 'bus.jpg' with arguments
model.predict(source="d.mp4", save=True, imgsz=320, conf=0.5)

# Configure the tracking parameters and run the tracker
model = YOLO('yolov8n.pt')

results = model.track(source="d.mp4",conf=0.3, iou=0.5, save=True, tracker="bytetrack.yaml")

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

# Set up video capture
cap = cv2.VideoCapture("d.mp4")

# Define the line coordinates
START = sv.Point(182, 254)
END = sv.Point(462, 254)

# Store the track history
track_history = defaultdict(lambda: [])

# Create a dictionary to keep track of objects that have crossed the line
crossed_objects = {}

# Open a video sink for the output video
video_info = sv.VideoInfo.from_video_path("d.mp4")
with sv.VideoSink("output_single_line.mp4", video_info) as sink:
    
    while cap.isOpened():
        success, frame = cap.read()

        if success:
            # Run YOLOv8 tracking on the frame, persisting tracks between frames
            results = model.track(frame, classes=[2, 3, 5, 7], persist=True, save=True, tracker="bytetrack.yaml")

            # Get the boxes and track IDs
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # Plot the tracks and count objects crossing the line
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                track = track_history[track_id]
                track.append((float(x), float(y)))  # x, y center point
                if len(track) > 30:  # retain 30 tracks for 30 frames
                    track.pop(0)

                # Check if the object crosses the line
                if START.x < x < END.x and abs(y - START.y) < 5:  # Assuming objects cross horizontally
                    if track_id not in crossed_objects:
                        crossed_objects[track_id] = True

                    # Annotate the object as it crosses the line
                    cv2.rectangle(annotated_frame, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)

            # Draw the line on the frame
            cv2.line(annotated_frame, (START.x, START.y), (END.x, END.y), (0, 255, 0), 2)

            # Write the count of objects on each frame
            count_text = f"Objects crossed: {len(crossed_objects)}"
            cv2.putText(annotated_frame, count_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Write the frame with annotations to the output video
            sink.write_frame(annotated_frame)
        else:
            break

# Release the video capture
cap.release()