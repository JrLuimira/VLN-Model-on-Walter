#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch
import numpy as np
from PIL import Image

# Para sincronizar múltiples tópicos de entrada (RGB y Depth)
import message_filters

# Importar las clases de los scripts de COW
# Asegúrate de que los archivos clip_owl.py y agent_fbe.py estén en tu PYTHONPATH
# import clip_owl # Reemplazar con la importación real
# import agent_fbe # Reemplazar con la importación real

# Simulación de importación para el ejemplo (asume que los archivos existen en el path)
# En un entorno real, simplemente importarías las clases.
#try:
from walter_scripts.localization.clip_owl import ClipOwl
from walter_scripts.agents.agent_mode import AgentMode
from walter_scripts.agents.agent_fbe import AgentFbe
    # Importamos la clase AgentFbe y AgentMode
#except ImportError:
    #print("Error: Make sure clip_owl.py and agent_fbe.py are available in your ROS package's source directory.")
    #exit()


# --- Configuración de Parámetros de Movimiento ---
# Define la velocidad lineal y angular para las acciones del robot.
LINEAR_SPEED = 0.2  # m/s
ANGULAR_SPEED = 0.5 # radians/s (equivale a ~28.6 grados/s)

# Asegúrate de que estos valores coincidan con los `forward_distance` y `rotation_degrees`
# definidos en `src/simulation/constants.py` del repositorio original de COW,
# o ajusta los parámetros del constructor de AgentFbe.

class AgentFbeRosNode(Node):

    def __init__(self):
        super().__init__('agent_fbe_node')

        # === 1. Inicialización de la Lógica del Agente (AgentFbe) ===
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.bridge = CvBridge()

        # Parámetros para la inicialización de ClipOwl
        clip_model_name = 'ViT-B/32'
        target_classes = ['car', 'person', 'traffic_light']
        classes_clip = ['vehicle', 'human', 'traffic_signal']
        templates = ['a photo of a {}']
        threshold = 0.5

        # Inicializar AgentFbe. AgentFbe internamente inicializará ClipOwl a través de:
        # self.clip_module = ClipOwl(...) (Esto no está en el código de FBE, sino en un hijo,
        # pero para el ejemplo, supongamos que ClipOwl se instancia aquí y se pasa)

        # Dado que AgentFbe es una clase abstracta (ABC), debemos crear una subclase
        # que implemente 'localize_object' si no lo hace, o instanciar una subclase concreta.
        # En el código de AgentFbe, `localize_object` usa `self.clip_module`.
        # Tendremos que inicializar `ClipOwl` por separado y asignarlo.

        # Vamos a inicializar la clase ClipOwl y luego la AgentFbe,
        # y asignar ClipOwl a AgentFbe (o subclase) si es necesario.
        # Si AgentFbe es abstracta, crearemos una subclase simple aquí.

        class ConcreteAgentFbe(AgentFbe):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                # En este punto, `self.clip_module` debe ser configurado por la subclase.
                # Para el ejemplo, lo pasaremos por fuera o en el constructor.
                # El código de COW original asume que self.clip_module se establece en un child.
                pass

            def localize_object(self, observations):
                # Implementación de localize_object que usa el clip_module
                img_tensor = None
                if self.transform is not None:
                    img_tensor = self.transform(observations["rgb"])
                else:
                    img_tensor = observations["rgb"]

                if torch.is_tensor(img_tensor):
                    img_tensor = img_tensor.unsqueeze(0)

                # self.clip_module (instancia de ClipOwl) se usa aquí.
                # observations["object_goal"] viene del nodo ROS.
                return self.clip_module(img_tensor, observations["object_goal"])


        # Inicializar el detector de objetos (ClipOwl)
        self.clip_detector = ClipOwl(
            clip_model_name=clip_model_name,
            classes=target_classes,
            classes_clip=classes_clip,
            templates=templates,
            threshold=threshold,
            device=self.device
        )

        # Inicializar AgentFbe (usando la subclase concreta)
        # Nota: Los parámetros fov, agent_height, etc. deben ser ajustados
        # para que coincidan con la configuración de tu robot y RealSense.
        self.agent_fbe = ConcreteAgentFbe(
            fov=90.0, # Ejemplo: Campo de visión horizontal de la cámara
            device=self.device,
            agent_height=1.0, # Altura del sensor en metros
            floor_tolerance=0.1, # Tolerancia para el suelo
            voxel_size_m=0.1, # Tamaño de voxel para el mapeo
            wandb_log=False
        )
        # Asignar el detector de objetos al agente.
        self.agent_fbe.clip_module = self.clip_detector
        self.get_logger().info("AgentFbe and ClipOwl initialized successfully.")


        # === 2. Configuración de ROS 2 Subscriber y Publisher ===
        # Publicador para el control de movimiento
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptores para los datos de la RealSense (RGB y Profundidad)
        rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw')

        # Sincronizador: Asegura que procesemos pares de imágenes (RGB y Profundidad)
        # tomadas en el mismo instante de tiempo.
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 10, 0.1) # Cola de 10, tolerancia de 0.1 segundos
        self.time_synchronizer.registerCallback(self.image_callback)

        self.get_logger().info("Subscribing to image topics and synchronizing.")

        # Parámetro del objeto a buscar (puedes cambiarlo dinámicamente)
        self.target_object = 'car'

    def image_callback(self, ros_rgb_msg, ros_depth_msg):
        """Callback function called when a synchronized pair of images arrives."""
        try:
            # === Paso 1: Conversión de ROS Image a Numpy/PIL ===
            # Convertir el mensaje de ROS Image (RGB) a un array de NumPy (OpenCV format)
            cv_rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb_msg, desired_encoding="rgb8")
            pil_rgb_image = Image.fromarray(cv_rgb_image)

            # Convertir el mensaje de ROS Image (Profundidad) a un array de NumPy
            cv_depth_image = self.bridge.imgmsg_to_cv2(ros_depth_msg, desired_encoding="passthrough")

            # === Paso 2: Preparar el diccionario de observaciones ===
            # Crear el diccionario 'observations' que espera AgentFbe.act()
            # AgentFbe necesita 'rgb' y 'object_goal'.
            # FrontierBasedExploration (self.fbe) dentro de AgentFbe usará la profundidad
            # internamente (dependiendo de cómo esté implementada).
            observations = {
                "rgb": pil_rgb_image,
                "depth": cv_depth_image, # Pasar la imagen de profundidad
                "object_goal": self.target_object # El objeto que buscamos
            }

            # === Paso 3: Llamar a la lógica de navegación (AgentFbe) ===
            # El método act() devuelve la acción de navegación ("Forward", "RotateLeft", etc.)
            action = self.agent_fbe.act(observations)

            # === Paso 4: Publicar la acción como Twist en /cmd_vel ===
            self.get_logger().info(f"Current action: {action}")
            twist_msg = self._convert_action_to_twist(action)
            self.cmd_vel_publisher.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image or generating action: {e}")

    def _convert_action_to_twist(self, action: str) -> Twist:
        """Converts the string action from AgentFbe to a ROS Twist message."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if action == "Forward":
            twist.linear.x = LINEAR_SPEED
        elif action == "RotateLeft":
            twist.angular.z = ANGULAR_SPEED
        elif action == "RotateRight":
            twist.angular.z = -ANGULAR_SPEED # Asume que "RotateRight" es un caso posible
        # Puedes añadir más acciones (e.g., "Stop") si tu AgentFbe las genera.

        return twist

def main(args=None):
    rclpy.init(args=args)
    agent_fbe_node = AgentFbeRosNode()
    rclpy.spin(agent_fbe_node)
    agent_fbe_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()