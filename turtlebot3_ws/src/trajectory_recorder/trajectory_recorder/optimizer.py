#!/usr/bin/env python3
"""
Selecciona y reproduce la mejor trayectoria grabada.

Busca todos los archivos JSON dentro de input_dir, asume que las rutas válidas
comienzan y terminan aproximadamente en el mismo lugar, calcula una métrica
para cada una y reproduce la mejor publicando sus cmd_vel en cmd_vel_topic.

Criterios disponibles:
- fastest: menor duración grabada.
- shortest: menor distancia recorrida.
- balanced: combina duración y distancia normalizadas.

Cada archivo debe contener muestras compatibles con recorder.py, por ejemplo:
{
  "t": 0.10,
  "cmd_vel": {"linear_x": 0.2, "angular_z": 0.0},
  "odom": {"x": 0.0, "y": 0.0, "yaw": 0.0},
  "distance": 0.02
}
"""

import glob
import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

from trajectory_recorder.utils import load_trajectory_json


class TrajectorySelectorPlayer(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_selector_player_node')

        self.declare_parameter(
            'input_dir', os.path.expanduser('~/turtlebot3_ws/trajectories')
        )
        self.declare_parameter('file_pattern', '*.json')
        self.declare_parameter('criterion', 'fastest')
        self.declare_parameter('start_tolerance', 0.50)
        self.declare_parameter('goal_tolerance', 0.50)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('auto_run', True)
        self.declare_parameter('exclude_selected_files', True)
        self.declare_parameter('selected_output_filename', 'selected_trajectory.json')

        self.input_dir = os.path.expanduser(
            str(self.get_parameter('input_dir').value)
        )
        self.file_pattern = str(self.get_parameter('file_pattern').value)
        self.criterion = str(self.get_parameter('criterion').value).lower()
        self.start_tolerance = float(self.get_parameter('start_tolerance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.playback_rate = max(
            0.01, float(self.get_parameter('playback_rate').value)
        )
        self.exclude_selected_files = bool(
            self.get_parameter('exclude_selected_files').value
        )
        self.selected_output_filename = str(
            self.get_parameter('selected_output_filename').value
        )

        if self.criterion not in {'fastest', 'shortest', 'balanced'}:
            raise ValueError(
                'criterion debe ser fastest, shortest o balanced'
            )

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.run_service = self.create_service(
            Trigger, 'select_and_play_trajectory', self._service_callback
        )

        self.samples: List[Dict[str, Any]] = []
        self.sample_index = 0
        self.playback_start_ns: Optional[int] = None
        self.first_sample_t = 0.0
        self.timer = None
        self.selected_path: Optional[str] = None

        self.get_logger().info(
            f'Buscando rutas en {self.input_dir}/{self.file_pattern}; '
            f'criterio={self.criterion}; salida={self.cmd_vel_topic}'
        )

        if bool(self.get_parameter('auto_run').value):
            try:
                self.select_and_play()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'No se pudo iniciar: {exc}')

    def _service_callback(self, request, response):
        del request
        try:
            selected, metrics = self.select_and_play()
            response.success = True
            response.message = (
                f'Seleccionada {os.path.basename(selected)}: '
                f'{metrics["duration"]:.3f} s, '
                f'{metrics["distance"]:.3f} m'
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def select_and_play(self) -> Tuple[str, Dict[str, float]]:
        routes = self._load_candidates()
        if not routes:
            raise FileNotFoundError(
                f'No hay trayectorias válidas en {self.input_dir}'
            )

        compatible = self._filter_compatible_routes(routes)
        if not compatible:
            raise RuntimeError(
                'Ninguna trayectoria cumple las tolerancias de inicio y meta'
            )

        selected = self._choose_best(compatible)
        self.selected_path = selected['path']
        self.samples = selected['samples']
        self._save_selected_route(selected)

        self.get_logger().info('Comparación de trayectorias:')
        for route in sorted(compatible, key=lambda item: item['score']):
            self.get_logger().info(
                f'  {os.path.basename(route["path"])} | '
                f't={route["duration"]:.3f}s | '
                f'd={route["distance"]:.3f}m | '
                f'score={route["score"]:.5f}'
            )

        self.get_logger().info(
            f'Seleccionada: {os.path.basename(self.selected_path)}'
        )
        self._start_playback()
        return self.selected_path, {
            'duration': selected['duration'],
            'distance': selected['distance'],
            'score': selected['score'],
        }

    def _load_candidates(self) -> List[Dict[str, Any]]:
        paths = sorted(glob.glob(os.path.join(self.input_dir, self.file_pattern)))
        routes: List[Dict[str, Any]] = []

        for path in paths:
            basename = os.path.basename(path)
            if self.exclude_selected_files and basename == self.selected_output_filename:
                continue
            if 'optimized' in basename.lower():
                continue

            try:
                samples = load_trajectory_json(path)
                if not isinstance(samples, list) or len(samples) < 2:
                    self.get_logger().warn(f'Omitiendo {basename}: ruta vacía/corta')
                    continue

                start = self._pose_xy(samples[0])
                goal = self._pose_xy(samples[-1])
                if start is None or goal is None:
                    self.get_logger().warn(f'Omitiendo {basename}: odometría inválida')
                    continue

                duration = self._duration(samples)
                distance = self._distance(samples)
                if duration <= 0.0:
                    self.get_logger().warn(f'Omitiendo {basename}: duración inválida')
                    continue

                routes.append({
                    'path': path,
                    'samples': samples,
                    'start': start,
                    'goal': goal,
                    'duration': duration,
                    'distance': distance,
                    'score': math.inf,
                })
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Omitiendo {basename}: {exc}')

        return routes

    def _filter_compatible_routes(
        self, routes: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        compatible = []
        for route in routes:
            compatible.append(route)
        return compatible
        
    def _choose_best(self, routes: List[Dict[str, Any]]) -> Dict[str, Any]:
        if self.criterion == 'fastest':
            for route in routes:
                route['score'] = route['duration']
        elif self.criterion == 'shortest':
            for route in routes:
                route['score'] = route['distance']
        else:
            min_duration = min(route['duration'] for route in routes)
            min_distance = min(route['distance'] for route in routes)
            for route in routes:
                time_ratio = route['duration'] / max(min_duration, 1e-9)
                distance_ratio = route['distance'] / max(min_distance, 1e-9)
                route['score'] = 0.7 * time_ratio + 0.3 * distance_ratio

        return min(routes, key=lambda route: route['score'])

    def _start_playback(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.sample_index = 0
        self.first_sample_t = self._sample_time(self.samples[0], fallback=0.0)
        self.playback_start_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.005, self._playback_tick)

    def _playback_tick(self) -> None:
        if self.playback_start_ns is None or not self.samples:
            return

        elapsed = (
            (self.get_clock().now().nanoseconds - self.playback_start_ns)
            / 1e9
            * self.playback_rate
        )

        while self.sample_index < len(self.samples):
            sample = self.samples[self.sample_index]
            target_time = (
                self._sample_time(sample, fallback=float(self.sample_index) * 0.1)
                - self.first_sample_t
            )
            if target_time > elapsed:
                break

            self.publisher.publish(self._twist_from_sample(sample))
            self.sample_index += 1

        if self.sample_index >= len(self.samples):
            self._stop_robot()
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.get_logger().info('Reproducción terminada.')

    def _save_selected_route(self, selected: Dict[str, Any]) -> None:
        output_path = os.path.join(
            self.input_dir, self.selected_output_filename
        )
        payload = {
            'selected_from': os.path.basename(selected['path']),
            'criterion': self.criterion,
            'duration': selected['duration'],
            'distance': selected['distance'],
            'samples': selected['samples'],
        }
        os.makedirs(self.input_dir, exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as file:
            json.dump(payload, file, indent=2, ensure_ascii=False)

    def _stop_robot(self) -> None:
        self.publisher.publish(Twist())

    @staticmethod
    def _twist_from_sample(sample: Dict[str, Any]) -> Twist:
        cmd = sample.get('cmd_vel', {}) or {}
        msg = Twist()

        linear = cmd.get('linear', {}) if isinstance(cmd.get('linear'), dict) else {}
        angular = cmd.get('angular', {}) if isinstance(cmd.get('angular'), dict) else {}

        msg.linear.x = float(cmd.get('linear_x', linear.get('x', 0.0)))
        msg.linear.y = float(cmd.get('linear_y', linear.get('y', 0.0)))
        msg.linear.z = float(cmd.get('linear_z', linear.get('z', 0.0)))
        msg.angular.x = float(cmd.get('angular_x', angular.get('x', 0.0)))
        msg.angular.y = float(cmd.get('angular_y', angular.get('y', 0.0)))
        msg.angular.z = float(cmd.get('angular_z', angular.get('z', 0.0)))
        return msg

    @staticmethod
    def _pose_xy(sample: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        odom = sample.get('odom', {}) or {}
        try:
            return float(odom['x']), float(odom['y'])
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _sample_time(sample: Dict[str, Any], fallback: float) -> float:
        for key in ('t', 'time', 'timestamp'):
            if key in sample:
                try:
                    return float(sample[key])
                except (TypeError, ValueError):
                    pass
        return fallback

    def _duration(self, samples: List[Dict[str, Any]]) -> float:
        start_t = self._sample_time(samples[0], 0.0)
        end_t = self._sample_time(samples[-1], float(len(samples) - 1) * 0.1)
        return max(0.0, end_t - start_t)

    @staticmethod
    def _distance(samples: List[Dict[str, Any]]) -> float:
        """Calcula la distancia real acumulada tramo a tramo entre posiciones
        consecutivas (odom_x, odom_y)."""
        total = 0.0
        previous = TrajectorySelectorPlayer._pose_xy(samples[0])

        for sample in samples[1:]:
            current = TrajectorySelectorPlayer._pose_xy(sample)
            if previous is not None and current is not None:
                # Suma el segmento euclidiano paso a paso
                total += TrajectorySelectorPlayer._euclidean(previous, current)
                previous = current

        # Si no había datos odom válidos, intenta usar el campo distance seguro
        if total == 0.0:
            try:
                start_d = float(samples[0].get('distance', 0.0))
                end_d = float(samples[-1].get('distance', 0.0))
                if end_d > start_d:
                    return end_d - start_d
            except (TypeError, ValueError):
                pass

        return total

    @staticmethod
    def _median_point(points: List[Tuple[float, float]]) -> Tuple[float, float]:
        xs = sorted(point[0] for point in points)
        ys = sorted(point[1] for point in points)
        middle = len(points) // 2
        if len(points) % 2:
            return xs[middle], ys[middle]
        return (
            (xs[middle - 1] + xs[middle]) / 2.0,
            (ys[middle - 1] + ys[middle]) / 2.0,
        )

    @staticmethod
    def _euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def destroy_node(self):
        self._stop_robot()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectorySelectorPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
