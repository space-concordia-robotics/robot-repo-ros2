# scripts/train_yolov8.py
from ultralytics import YOLO

DATA = "datasets/ground_objects/data.yaml"
WEIGHTS = "yolov8n.pt"  # try yolov8s.pt if you have GPU headroom
EPOCHS = 150

model = YOLO(WEIGHTS)
model.train(
    data=DATA,
    epochs=EPOCHS,
    imgsz=640,
    batch=-1,
    device=0,       # or 'cpu'
    patience=30,
    cos_lr=True,
    mosaic=1.0, mixup=0.1,
    hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,
    degrees=5, translate=0.05, scale=0.5, shear=1.0, fliplr=0.5, flipud=0.0,
)
print("Best weights:", "runs/detect/train/weights/best.pt")
