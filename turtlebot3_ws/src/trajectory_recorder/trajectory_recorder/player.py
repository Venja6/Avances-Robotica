#!/usr/bin/env python3
"""
player.py
---------
Nodo ROS 2 que reproduce una trayectoria previamente grabada con
recorder.py, publicando comandos de velocidad en /cmd_vel.

Modo de operación:
* "open-loop": reproduce exactamente los cmd_vel grabados, sin usar
  realimentación de odometría.
* "closed-loop" (use_pid=True, por defecto): además de reproducir el
  cmd_vel grabado como término "feed-forward", corrige la trayectoria
  comparando la pose real (odometría actual) contra la pose que el
  robot tenía en el mismo instante durante la grabación, y añade una
  corrección PID sobre el error de distancia y el error de orientación.

Parámetros:
* filename        (str)   - archivo JSON a reproducir. Default: 'trajectory.json'
* input_dir       (str)   - carpeta donde buscar el archivo.
                            Default: '~/turtlebot3_ws/trajectories'
* playback_rate   (float) - frecuencia de publicación en Hz. Default: 10.0
* use_pid         (bool)  - activa la corrección por PID. Default: True
* kp_linear/ki_linear/kd_linear   - ganancias PID para el error de distancia.
* kp_angular/ki_angular/kd_angular - ganancias PID para el error angular.
* max_linear_correction  (float) - límite de corrección lineal (m/s). Default: 0.1
* max_angular_correction (float) - límite de corrección angular (rad/s). Default: 0.5
* cmd_vel_topic / odom_topic - tópicos usados. Defaults: '/cmd_vel', '/odom'
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from trajectory_recorder.utils import (
    quaternion_to_yaw,
    normalize_angle,
    euclidean_distance,
    load_trajectory_json,
)
from trajectory_recorder.pid import PIDController


class TrajectoryPlayer(Node):

    def __init__(self):
        super().__init__('trajectory_player_node')

        # --- Declaración de parámetros ---
        self.declare_parameter('filename', 'trajectory.json')
        self.declare_parameter(
            'input_dir',
            os.path.expanduser('~/turtlebot3_ws/trajectories')
        )
        self.declare_parameter('playback_rate', 10.0)
        self.declare_parameter('use_pid', True)

        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.05)

        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)

        self.declare_parameter('max_linear_correction', 0.1)
        self.declare_parameter('max_angular_correction', 0.5)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.filename = self.get_parameter('filename').value
        self.input_dir = self.get_parameter('input_dir').value
        self.playback_rate = float(self.get_parameter('playback_rate').value)
        self.use_pid = bool(self.get_parameter('use_pid').value)

        if self.playback_rate <= 0.0:
            self.get_logger().warn(
                'playback_rate <= 0, se usará 10.0 Hz por defecto.'
            )
            self.playback_rate = 10.0

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        max_lin_corr = float(self.get_parameter('max_linear_correction').value)
        max_ang_corr = float(self.get_parameter('max_angular_correction').value)

        # --- Controladores PID (uno para distancia, otro para yaw) ---
        self.pid_linear = PIDController(
            kp=float(self.get_parameter('kp_linear').value),
            ki=float(self.get_parameter('ki_linear').value),
            kd=float(self.get_parameter('kd_linear').value),
            output_limit=max_lin_corr,
            integral_limit=max_lin_corr * 5.0 if max_lin_corr > 0 else 1.0,
        )
        self.pid_angular = PIDController(
            kp=float(self.get_parameter('kp_angular').value),
            ki=float(self.get_parameter('ki_angular').value),
            kd=float(self.get_parameter('kd_angular').value),
            output_limit=max_ang_corr,
            integral_limit=max_ang_corr * 5.0 if max_ang_corr > 0 else 1.0,
        )

        # --- Carga de la trayectoria ---
        full_path = os.path.join(self.input_dir, self.filename)
        self.trajectory = load_trajectory_json(full_path)
        if not self.trajectory:
            self.get_logger().error(
                f'La trayectoria cargada desde {full_path} está vacía.'
            )
        else:
            self.get_logger().info(
                f'Trayectoria cargada: {len(self.trajectory)} muestras '
                f'desde {full_path}.'
            )

        # --- Estado interno ---
        self._index = 0
        self._current_odom = None
        self._has_odom = False
        self._finished = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # --- Pub/Sub ---
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos
        )

        # --- Timer de reproducción ---
        period = 1.0 / self.playback_rate
        self.timer = self.create_timer(period, self._playback_callback)

        mode = 'closed-loop (PID)' if self.use_pid else 'open-loop'
        self.get_logger().info(
            f'Reproduciendo trayectoria en modo {mode} a '
            f'{self.playback_rate} Hz sobre "{cmd_vel_topic}".'
        )

    # ------------------------------------------------------------------
    # Callback de odometría
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self._current_odom = {'x': pos.x, 'y': pos.y, 'yaw': yaw}
        self._has_odom = True

    # ------------------------------------------------------------------
    # Reproducción periódica
    # ------------------------------------------------------------------
    def _playback_callback(self) -> None:
        if self._finished:
            return

        if self._index >= len(self.trajectory):
            self._stop_robot()
            self._finished = True
            self.get_logger().info(
                'Reproducción de la trayectoria finalizada.'
            )
            return

        sample = self.trajectory[self._index]
        cmd = sample.get('cmd_vel', {})
        recorded_odom = sample.get('odom', {})

        linear_x = float(cmd.get('linear_x', 0.0))
        angular_z = float(cmd.get('angular_z', 0.0))

        if self.use_pid and self._has_odom and recorded_odom:
            linear_x, angular_z = self._apply_pid_correction(
                linear_x, angular_z, recorded_odom
            )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        self._index += 1

    # ------------------------------------------------------------------
    # Corrección PID basada en el error contra la odometría grabada
    # ------------------------------------------------------------------
    def _apply_pid_correction(
        self,
        linear_x: float,
        angular_z: float,
        recorded_odom: dict,
    ) -> tuple:
        """
        Compara la pose actual (self._current_odom) contra la pose que el
        robot tenía en la grabación en este mismo índice de la trayectoria,
        y añade una corrección PID sobre:

        * el error de distancia (proyectado sobre el eje de avance -> vel. lineal)
        * el error angular / de rumbo (-> vel. angular)

        La corrección se suma al comando "feed-forward" grabado, y se
        satura según los límites max_linear_correction / max_angular_correction
        configurados en los PID.
        """
        dt = 1.0 / self.playback_rate

        cur_x = self._current_odom['x']
        cur_y = self._current_odom['y']
        cur_yaw = self._current_odom['yaw']

        rec_x = float(recorded_odom.get('x', cur_x))
        rec_y = float(recorded_odom.get('y', cur_y))
        rec_yaw = float(recorded_odom.get('yaw', cur_yaw))

        # Error de orientación: diferencia entre el yaw grabado y el actual.
        yaw_error = normalize_angle(rec_yaw - cur_yaw)

        # Error de posición proyectado en el sistema de referencia del
        # robot (para saber si vamos "atrasados" o "adelantados" en el
        # eje de avance).
        dx = rec_x - cur_x
        dy = rec_y - cur_y
        distance_error = euclidean_distance(cur_x, cur_y, rec_x, rec_y)

        # Signo del error de distancia: positivo si el punto grabado está
        # "delante" del robot (proyección del vector de error sobre el
        # heading actual del robot); negativo si quedó "atrás".
        robot_heading_dot = dx * _cos(cur_yaw) + dy * _sin(cur_yaw)
        signed_distance_error = (
            distance_error if robot_heading_dot >= 0 else -distance_error
        )

        linear_correction = self.pid_linear.update(signed_distance_error, dt)
        angular_correction = self.pid_angular.update(yaw_error, dt)

        corrected_linear = linear_x + linear_correction
        corrected_angular = angular_z + angular_correction

        return corrected_linear, corrected_angular

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def _stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())
        self.pid_linear.reset()
        self.pid_angular.reset()


def _cos(angle: float) -> float:
    import math
    return math.cos(angle)


def _sin(angle: float) -> float:
    import math
    return math.sin(angle)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
