#!/usr/bin/env python3
"""
recorder.py
-----------
Nodo ROS 2 que graba la trayectoria de un TurtleBot3:

* Se suscribe a /cmd_vel (geometry_msgs/Twist) para registrar las
  velocidades comandadas.
* Se suscribe a /odom (nav_msgs/Odometry) para registrar la pose real
  (x, y, yaw) y las velocidades reales del robot.
* A una frecuencia configurable (record_rate), guarda una muestra
  combinada {t, cmd_vel, odom} en un buffer interno.
* Al finalizar (Ctrl+C) o al invocar el servicio "save_trajectory",
  vuelca el buffer completo a un archivo JSON.

Parámetros (declarados como parámetros ROS 2):
* filename      (str)   - nombre del archivo de salida. Default: 'trajectory.json'
* output_dir    (str)   - carpeta donde se guarda el archivo.
                          Default: '~/turtlebot3_ws/trajectories'
* record_rate   (float) - frecuencia de muestreo en Hz. Default: 10.0
* cmd_vel_topic (str)   - topico de velocidad comandada. Default: '/cmd_vel'
* odom_topic    (str)   - topico de odometria. Default: '/odom'
* motion_threshold (float) - umbral en m/s (lineal) y rad/s (angular) para
                             clasificar el movimiento como "straight",
                             "turn", "curve" o "stop". Default: 0.05

Cada muestra grabada incluye, además de {t, cmd_vel, odom}:
* segment_id     - identificador incremental de segmento; cambia cada vez
                    que el tipo de movimiento (motion_type) cambia respecto
                    a la muestra anterior.
* motion_type    - "straight" | "turn" | "curve" | "stop".
* distance       - distancia euclidiana recorrida (odometría) acumulada
                    desde el inicio de la grabación hasta esta muestra.

Estos campos son los que consume el nodo optimizer.py para resumir la
trayectoria en segmentos {segment_id, x, y, yaw, distancia recorrida}.
"""

import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from trajectory_recorder.utils import (
    quaternion_to_yaw,
    euclidean_distance,
    save_trajectory_json,
)


class TrajectoryRecorder(Node):

    def __init__(self):
        super().__init__('trajectory_recorder_node')

        # --- Declaración de parámetros ---
        self.declare_parameter('filename', 'trajectory.json')
        self.declare_parameter(
            'output_dir',
            os.path.expanduser('~/turtlebot3_ws/trajectories')
        )
        self.declare_parameter('record_rate', 10.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('motion_threshold', 0.05)

        requested_filename = str(self.get_parameter('filename').value)

        # Si se usa el nombre por defecto, genera uno único por ejecución
        # para no sobrescribir grabaciones anteriores.
        if requested_filename == 'trajectory.json':
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.filename = f'trajectory_{timestamp}.json'
        else:
            self.filename = requested_filename
        self.output_dir = self.get_parameter('output_dir').value
        self.record_rate = float(self.get_parameter('record_rate').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.motion_threshold = float(
            self.get_parameter('motion_threshold').value
        )

        if self.record_rate <= 0.0:
            self.get_logger().warn(
                'record_rate <= 0, se usará 10.0 Hz por defecto.'
            )
            self.record_rate = 10.0

        # --- Estado interno ---
        self._buffer = []
        self._start_time = None
        self._last_cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0}
        self._last_odom = None
        self._has_odom = False

        # Estado para segmentación por tipo de movimiento y distancia
        # acumulada (odometría).
        self._segment_id = 0
        self._last_motion_type = None
        self._total_distance = 0.0
        self._last_position = None  # (x, y) de la muestra anterior

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # --- Suscripciones ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos
        )

        # --- Servicio para guardar bajo demanda ---
        self.save_srv = self.create_service(
            Trigger, 'save_trajectory', self._save_service_callback
        )

        # --- Timer de muestreo ---
        period = 1.0 / self.record_rate
        self.timer = self.create_timer(period, self._sample_callback)

        self.get_logger().info(
            f'Grabando trayectoria: cmd_vel="{cmd_vel_topic}", '
            f'odom="{odom_topic}", frecuencia={self.record_rate} Hz. '
            f'Se guardará en: {os.path.join(self.output_dir, self.filename)}'
        )
        self.get_logger().info(
            'Presiona Ctrl+C para detener y guardar, o llama al '
            'servicio "save_trajectory" para guardar sin detener el nodo.'
        )

    # ------------------------------------------------------------------
    # Clasificación del tipo de movimiento
    # ------------------------------------------------------------------
    def _get_motion_type(self) -> str:
        """
        Clasifica el movimiento actual según el último cmd_vel recibido,
        comparando contra self.motion_threshold (m/s para lineal, rad/s
        para angular).
        """
        linear = abs(self._last_cmd_vel['linear_x'])
        angular = abs(self._last_cmd_vel['angular_z'])
        th = self.motion_threshold

        if linear > th and angular <= th:
            return 'straight'
        elif angular > th and linear <= th:
            return 'turn'
        elif linear > th and angular > th:
            return 'curve'
        else:
            return 'stop'

    # ------------------------------------------------------------------
    # Callbacks de suscripción
    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_vel = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
        }

    def _odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        self._last_odom = {
            'x': pos.x,
            'y': pos.y,
            'yaw': yaw,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z,
        }
        self._has_odom = True

    # ------------------------------------------------------------------
    # Muestreo periódico
    # ------------------------------------------------------------------
    def _sample_callback(self) -> None:
        if not self._has_odom:
            # Esperamos a tener al menos una lectura de odometría antes
            # de empezar a grabar muestras útiles.
            return

        if self._start_time is None:
            self._start_time = time.time()

        t = time.time() - self._start_time

        # --- Distancia acumulada (odometría) ---
        cur_x = self._last_odom['x']
        cur_y = self._last_odom['y']
        if self._last_position is not None:
            self._total_distance += euclidean_distance(
                self._last_position[0], self._last_position[1], cur_x, cur_y
            )
        self._last_position = (cur_x, cur_y)

        # --- Segmentación por tipo de movimiento ---
        motion_type = self._get_motion_type()
        if (
            self._last_motion_type is not None
            and motion_type != self._last_motion_type
        ):
            self._segment_id += 1
        self._last_motion_type = motion_type

        sample = {
            't': round(t, 4),
            'segment_id': self._segment_id,
            'motion_type': motion_type,
            'distance': round(self._total_distance, 5),
            'cmd_vel': dict(self._last_cmd_vel),
            'odom': dict(self._last_odom),
        }
        self._buffer.append(sample)

    # ------------------------------------------------------------------
    # Guardado
    # ------------------------------------------------------------------
    def _save_service_callback(self, request, response):
        try:
            self.save_to_disk()
            response.success = True
            response.message = (
                f'Trayectoria guardada ({len(self._buffer)} muestras) en '
                f'{os.path.join(self.output_dir, self.filename)}'
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Error al guardar: {exc}'
        return response

    def save_to_disk(self) -> None:
        full_path = os.path.join(self.output_dir, self.filename)
        save_trajectory_json(full_path, self._buffer)
        self.get_logger().info(
            f'Guardadas {len(self._buffer)} muestras en {full_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al salir (Ctrl+C) guardamos siempre lo grabado hasta el momento.
        node.save_to_disk()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
