import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class FakeDepthNode(Node):
    def __init__(self):
        super().__init__('fake_depth_node')
        
        # Suscriptor a la cámara RGB
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',  # Cambia esto al tópico real de tu cámara
            self.image_callback,
            10)
        
        # Publicador de la Falsa Profundidad
        self.publisher_ = self.create_publisher(Image, '/camera/fake_depth', 10)
        
        self.bridge = CvBridge()
        
        # Parámetros de calibración básica (en metros)
        self.distancia_minima = 0.2  # Distancia al suelo cerca del robot
        self.distancia_maxima = 1.5  # Distancia máxima que alcanza a ver la cámara
        
        self.get_logger().info('Nodo de Falsa Profundidad Iniciado')

    def image_callback(self, msg):
        # 1. Convertir mensaje de ROS a imagen de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Convertir a escala de grises
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 3. Umbral para detectar líneas negras
        # Ajusta el '50' según la iluminación. Píxeles < 50 se vuelven 255 (blanco)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # 4. Crear una imagen de fondo con "profundidad infinita" (ej. 5.0 metros)
        # depthimage_to_laserscan requiere flotantes de 32 bits (float32)
        alto, ancho = thresh.shape
        fake_depth = np.full((alto, ancho), 5.0, dtype=np.float32)
        
        # 5. Mapear píxeles de la línea negra a una distancia estimada
        # Las filas de abajo de la imagen están más cerca del robot; las de arriba, más lejos.
        for fila in range(alto):
            # Calculamos una distancia interpolada linealmente por cada fila
            distancia_estimada = self.distancia_maxima - (fila / alto) * (self.distancia_maxima - self.distancia_minima)
            
            # Donde la línea sea blanca en 'thresh', asignamos la distancia calculada
            fake_depth[fila, thresh[fila] == 255] = distancia_estimada

        # 6. Convertir de vuelta a mensaje de ROS (formato 32FC1: 32-bit Float 1 Channel)
        depth_msg = self.bridge.cv2_to_imgmsg(fake_depth, encoding='32FC1')
        depth_msg.header = msg.header  # Mantener la misma estampa de tiempo y frame_id
        
        # 7. Publicar
        self.publisher_.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
