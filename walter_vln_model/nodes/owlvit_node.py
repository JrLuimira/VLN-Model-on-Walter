#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch
import numpy as np
from PIL import Image as PILImage

import spacy
from transformers import OwlViTProcessor, OwlViTForObjectDetection

class OwlVitNode(Node):
    def __init__(self):
        super().__init__("owlvit_node")

        self.bridge = CvBridge()

        self.detect_pub = self.create_publisher(String, "/owlvit/deteccion", 10)


        # Suscripción a la cámara RealSense
        self.subscription = self.create_subscription(
            Image,
            "/camera/rs_d435/color/image_raw",
            self.image_callback,
            10
        )

        # Cargar SpaCy
        self.get_logger().info("Cargando modelo SpaCy...")
        self.nlp = spacy.load("en_core_web_sm")

        # Conectar GPU si existe
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Cargar OwlViT
        self.get_logger().info("Cargando OwlViT...")
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32").to(self.device)

        # Texto de búsqueda
        self.text_queries = ["Find a bottle that is behind a lamp."]
        self.get_logger().info(f"El texto ingresado es: {self.text_queries}")
        self.cleaned_queries = [self.clean_query_spacy(q) for q in self.text_queries]

        self.get_logger().info(f"Texto procesado: {self.cleaned_queries}")

    def clean_query_spacy(self, q: str) -> str:
        doc = self.nlp(q.lower().strip())
        chunks = sorted(doc.noun_chunks, key=lambda c: len(c.text), reverse=True)
        if chunks:
            return chunks[0].text.strip()
        nouns = [t.text for t in doc if t.pos_ in ("NOUN", "PROPN")]
        return " ".join(nouns).strip() or "object"

    def image_callback(self, msg):
        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert OpenCV → PIL
        pil_img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        target_sizes = torch.tensor([pil_img.size[::-1]])

        # Procesar inputs
        inputs = self.processor(
            text=self.cleaned_queries,
            images=pil_img,
            return_tensors="pt"
        ).to(self.device)

        # Inferencia
        with torch.no_grad():
            outputs = self.model(**inputs)

        # Post-procesamiento OwlViT
        results = self.processor.post_process_grounded_object_detection(
            outputs=outputs,
            target_sizes=target_sizes,
            threshold=0.6,
            text_labels=[self.cleaned_queries]
        )[0]

        boxes = results["boxes"]
        scores = results["scores"]
        labels = results["labels"]

        detected_flag = False

        # Dibujar bounding boxes
        annotated = frame.copy()

        for box, score, label_idx in zip(boxes, scores, labels):
            box = [int(i) for i in box.tolist()]
            x1, y1, x2, y2 = box

            label_text = self.cleaned_queries[label_idx]
            text = f"{label_text}: {score.item():.2f}"

            # Rectángulo
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)

            # Texto
            cv2.putText(annotated, text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        

        if len(scores) > 0:
            self.get_logger().info(f"score[0]={float(scores[0].item()):.3f}")

            msg_out = String()
            msg_out.data = "Si"
            self.detect_pub.publish(msg_out)

        else:
            self.get_logger().info("Sin detecciones")
            msg_out = String()
            msg_out.data = "No"
            self.detect_pub.publish(msg_out)
        

        # Mostrar en ventana OpenCV
        cv2.imshow("OWL-ViT Detection", annotated)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    OwlVit_node = OwlVitNode()
    rclpy.spin(OwlVit_node)
    OwlVit_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
