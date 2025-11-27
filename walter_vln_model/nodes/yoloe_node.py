#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import torch
import numpy as np

from ultralytics import YOLO


class YOLOENode(Node):
    def __init__(self):
        super().__init__('yoloe_node')

        # -----------------------
        # Parámetros ROS
        # -----------------------
        self.declare_parameter('image_topic', '/camera/rs_d435/color/image_raw')
        # Pesos YOLOE publicados en HF, formateados para Ultralytics:
        # https://huggingface.co/jameslahm/yoloe
        self.declare_parameter('model_name', 'jameslahm/yoloe-v8l-seg')
        self.declare_parameter('conf_threshold', 0.25)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').value

        # -----------------------
        # Objeto puntual a detectar
        # -----------------------
        # Nombre de clase COCO. ¡NOTA IMPORTANTE! "human face" NO es una clase COCO.
        # Hemos cambiado a "person" (persona) para que funcione con el modelo.
        self.target_class = "bottle" 

        # -----------------------
        # Dispositivo
        # -----------------------
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Usando dispositivo: {self.device}")

        # -----------------------
        # Cargar modelo YOLOE (vía Ultralytics)
        # -----------------------
        self.get_logger().info(f"Cargando modelo YOLOE (Ultralytics): {model_name}")
        # Mapea el modelo al dispositivo (si no está implícito en la carga)
        self.model = YOLO(model_name).to(self.device) 

        # Utilidades
        self.bridge = CvBridge()

        # Suscripción cámara
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Publicaciones
        self.pub_flag = self.create_publisher(String, 'yoloe/object_found', 10)
        self.pub_image = self.create_publisher(Image, 'yoloe/annotated_image', 10)

        self.get_logger().info(f"Suscrito a imagen: {image_topic}")
        self.get_logger().info(f"Detectando SOLO: {self.target_class}")
        # Inicializar ventana CV2
        cv2.namedWindow("YOLOE Detection", cv2.WINDOW_AUTOSIZE)

    # --------------------------------------------------------
    # CALLBACK DE IMAGEN
    # --------------------------------------------------------
    def image_callback(self, msg: Image):

        # Convertir ROS2 → OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        # Inferencia con YOLOE (API de Ultralytics)
        # Aseguramos que la inferencia se realice en el dispositivo correcto si el modelo no lo gestionó
        results = self.model(cv_image, conf=self.conf_threshold, device=self.device, verbose=False)

        detected = False
        annotated = cv_image.copy()
        
        # Con Ultralytics, results siempre es una lista, aunque esté vacía
        if not results:
            self._publish_outputs(detected, annotated, msg)
            return

        r = results[0]
        
        # Contador de detecciones encontradas y filtradas
        filtered_detections_count = 0 

        if hasattr(r, "boxes") and r.boxes is not None:
            # Transferir resultados a CPU para su posterior manejo con NumPy/OpenCV
            boxes_xyxy = r.boxes.xyxy.cpu().numpy()
            classes = r.boxes.cls.cpu().numpy()
            scores = r.boxes.conf.cpu().numpy()
            names = r.names  # dict: id -> class_name

            for box, cls_id, score in zip(boxes_xyxy, classes, scores):
                class_name = names.get(int(cls_id), "unknown")

                # -----------------------
                # FILTRO: SOLO CLASE OBJETIVO
                # -----------------------
                if class_name.lower() != self.target_class.lower():
                    continue

                detected = True
                filtered_detections_count += 1
                
                x1, y1, x2, y2 = map(int, box.tolist())
                label = f"{class_name}: {score:.2f}"

                # Dibujar
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )
            
            if filtered_detections_count > 0:
                 self.get_logger().info(f"YOLOE: {filtered_detections_count} caja(s) de '{self.target_class}' detectada(s)")


        # Publicar resultados
        self._publish_outputs(detected, annotated, msg)

    # --------------------------------------------------------
    # Helper para publicar outputs
    # --------------------------------------------------------
    def _publish_outputs(self, detected: bool, annotated, msg_in: Image):
        # Flag "Si" / "No"
        out = String()
        out.data = "Si" if detected else "No"
        self.pub_flag.publish(out)

        # Imagen anotada (si hay alguien suscrito)
        if self.pub_image.get_subscription_count() > 0:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = msg_in.header
            self.pub_image.publish(img_msg)

        # Debug visual
        cv2.imshow("YOLOE Detection", annotated)
        # La llamada a waitKey(1) es esencial para que imshow funcione
        cv2.waitKey(1) 


def main(args=None):
    rclpy.init(args=args)
    node = YOLOENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Cerrar todas las ventanas de OpenCV al salir
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()