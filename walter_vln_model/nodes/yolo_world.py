#!/usr/bin/env python3

from ultralytics import YOLOWorld
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
import numpy as np

class YoloWorldNode(Node):
    def __init__(self):
        super().__init__("yoloworld_node")

        self.bridge = CvBridge()
        self.device = "cuda:0"

        # Clases que quieres buscar (prompt textual)
        self.target_classes = ["face","bottle","book"]  # o ["person", "face"], etc.

        self.get_logger().info(f"Usando dispositivo: {self.device}")
        self.get_logger().info("Cargando modelo YOLO-World: yolov8s-world.pt")

        self.model = YOLOWorld("yolov8s-world.pt")
        self.model.to(self.device)

        # Fijar las clases (esto genera los embeddings de texto offline)
        self.model.set_classes(self.target_classes)

        self.detect_pub = self.create_publisher(String, "/yoloworld/deteccion", 10)

        self.subscription = self.create_subscription(
            Image,
            "/camera/rs_d435/color/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("Suscrito a /camera/rs_d435/color/image_raw")
        self.get_logger().info(f"Detectando SOLO: {self.target_classes}")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model.predict(
            source=frame,
            device=self.device,
            conf=0.60,
            verbose=False
        )[0]

        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        cls_ids = results.boxes.cls.cpu().numpy().astype(int)

        detected_flag = False
        annotated = frame.copy()

        for box, score, cls_id in zip(boxes, scores, cls_ids):
            if score < 0.25:
                continue

            class_name = self.model.names[cls_id]  # debería coincidir con tus prompts

            # Si solo te interesa saber si hay alguna de las target_classes
            if class_name in self.target_classes:
                detected_flag = True

            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(
                annotated,
                f"{class_name}: {score:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        out = String()
        out.data = "Si" if detected_flag else "No"
        self.detect_pub.publish(out)
        # opcional
        # self.get_logger().info(f"Deteccion {self.target_classes}: {out.data}")
        cv2.imshow("Yolo-World Detection", annotated)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    YW_node = YoloWorldNode()
    rclpy.spin(YW_node)
    YW_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()