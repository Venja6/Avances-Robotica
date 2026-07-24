"""
utils.py
--------
Funciones auxiliares compartidas entre el grabador (recorder) y el
reproductor (player) de trayectorias:

* Conversión de cuaternión -> yaw (ángulo de guiñada, 2D).
* Normalización de ángulos al rango [-pi, pi].
* Carga y guardado de trayectorias en formato JSON.
* Cálculo de distancia euclidiana entre dos puntos.
"""

import json
import math
import os
from typing import Any, Dict, List


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Convierte un cuaternión (x, y, z, w) al ángulo de yaw (rotación en Z),
    válido para robots que se mueven en el plano 2D (caso TurtleBot3).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo en radianes al rango [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Distancia euclidiana entre dos puntos (x1,y1) y (x2,y2)."""
    return math.hypot(x2 - x1, y2 - y1)


def save_trajectory_json(path: str, data: List[Dict[str, Any]]) -> None:
    """
    Guarda la trayectoria grabada como una lista de diccionarios en un
    archivo JSON legible. Crea el directorio de destino si no existe.
    """
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)

    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'trajectory': data}, f, indent=2)


def load_trajectory_json(path: str) -> List[Dict[str, Any]]:
    """
    Carga una trayectoria previamente grabada desde un archivo JSON.
    Lanza FileNotFoundError si el archivo no existe.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"No se encontró el archivo de trayectoria: {path}"
        )

    with open(path, 'r', encoding='utf-8') as f:
        content = json.load(f)

    return content.get('trajectory', [])


def save_waypoints_json(path: str, data: List[Dict[str, Any]]) -> None:
    """
    Guarda una lista de waypoints (salida del optimizer, con campos
    segment_id, x, y, yaw, distance, motion_type) en un archivo JSON.
    Usa la clave 'waypoints' para no confundirla con una trayectoria
    cruda grabada por recorder.py (clave 'trajectory').
    """
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)

    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'waypoints': data}, f, indent=2)


def load_waypoints_json(path: str) -> List[Dict[str, Any]]:
    """Carga una lista de waypoints previamente optimizada."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"No se encontró el archivo de waypoints: {path}"
        )

    with open(path, 'r', encoding='utf-8') as f:
        content = json.load(f)

    return content.get('waypoints', [])


def _point_line_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Distancia perpendicular del punto P a la recta A-B (usada por RDP)."""
    if (ax, ay) == (bx, by):
        return euclidean_distance(px, py, ax, ay)

    num = abs((by - ay) * px - (bx - ax) * py + bx * ay - by * ax)
    den = math.hypot(by - ay, bx - ax)
    return num / den


def douglas_peucker(
    points: List[Dict[str, float]], epsilon: float
) -> List[Dict[str, float]]:
    """
    Simplifica una polilínea de puntos [{'x':.., 'y':.., ...}, ...] con el
    algoritmo de Ramer-Douglas-Peucker, conservando el punto de mayor
    desviación en cada tramo mientras esa desviación supere `epsilon`
    (metros). Conserva siempre el primer y el último punto.
    Si epsilon <= 0 o hay menos de 3 puntos, retorna la lista sin cambios.
    """
    if epsilon <= 0.0 or len(points) < 3:
        return list(points)

    first, last = points[0], points[-1]
    max_dist = 0.0
    index = 0

    for i in range(1, len(points) - 1):
        d = _point_line_distance(
            points[i]['x'], points[i]['y'],
            first['x'], first['y'],
            last['x'], last['y'],
        )
        if d > max_dist:
            max_dist = d
            index = i

    if max_dist > epsilon:
        left = douglas_peucker(points[:index + 1], epsilon)
        right = douglas_peucker(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [first, last]
