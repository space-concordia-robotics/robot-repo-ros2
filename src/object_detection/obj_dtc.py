#!/usr/bin/env python3
# obj_dtc.py
import argparse
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self, weights, conf, iou, camera_topic, class_filter):
        super().__init__('yolov8_detector')
        self.bridge = CvBridge()
        self.model = YOLO(weights)
        self.conf = conf
        self.iou = iou
        self.class_filter = set(class_filter) if class_filter else None

        self.sub = self.create_subscription(Image, camera_topic, self.image_cb, 10)
        self.pub_det = self.create_publisher(Detection2DArray, "~/detections", 10)
        self.pub_img = self.create_publisher(Image, "~/image_with_detections", 10)

        names = self.model.names
        self.names = {int(k): v for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))
        self.get_logger().info(f"Loaded YOLOv8 model: {weights}")
        self.get_logger().info(f"Classes: {self.names}")

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Run inference
        results = self.model.predict(source=frame, conf=self.conf, iou=self.iou, imgsz=640, verbose=False)
        if not results:
            return
        r = results[0]

        det_arr = Detection2DArray()
        det_arr.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)

        # Draw and build messages
        im_anno = r.plot()  # ultralytics draws boxes/labels

        for b in r.boxes:
            cls_id = int(b.cls[0])
            score = float(b.conf[0])
            cls_name = self.names.get(cls_id, str(cls_id))

            if self.class_filter and cls_name not in self.class_filter:
                continue

            # xyxy -> center/size
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = (x2 - x1)
            h = (y2 - y1)

            det = Detection2D()
            det.header = det_arr.header

            # hypothesis
            hyp = ObjectHypothesisWithPose()
            hyp.id = str(cls_id)              # numeric ID as string
            hyp.score = score                 # confidence
            det.results.append(hyp)

            # bbox
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            # Put class name into the detection id field (common convenience)
            det.id = cls_name
            det_arr.detections.append(det)

        # Publish detections
        self.pub_det.publish(det_arr)

        # Publish debug image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(im_anno, encoding="bgr8")
            out_msg.header = msg.header
            self.pub_img.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=str, default="models/ground_objects_best.pt")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--camera-topic", type=str, default="/camera/image_raw")
    parser.add_argument("--classes", type=str, nargs="*", default=[], help="Optional whitelist, e.g. --classes hammer water_bottle")
    args, unknown = parser.parse_known_args()

    rclpy.init(args=None)
    node = YoloDetectorNode(
        weights=args.weights,
        conf=args.conf,
        iou=args.iou,
        camera_topic=args.camera_topic,
        class_filter=args.classes
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
