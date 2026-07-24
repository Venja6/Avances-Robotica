# trajectory_recorder

Paquete ROS 2 (Humble) para **grabar** y **reproducir** trayectorias en un
TurtleBot3, con corrección de trayectoria mediante control **PID** basado
en odometría.

## Estructura

```
trajectory_recorder/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/trajectory_recorder
├── trajectory_recorder/
│   ├── __init__.py
│   ├── recorder.py      # nodo grabador (cmd_vel + odom -> JSON)
│   ├── player.py         # nodo reproductor (JSON -> cmd_vel, con PID)
│   ├── pid.py             # controlador PID reutilizable
│   └── utils.py           # helpers (cuaterniones, JSON, ángulos)
└── launch/
    ├── record.launch.py
    └── play.launch.py
```

## Instalación

Copia la carpeta `trajectory_recorder/` dentro de `~/turtlebot3_ws/src/` y
compila:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select trajectory_recorder
source install/setup.bash
```

## Uso

### 1. Grabar una trayectoria

En una terminal (con el robot o la simulación de TurtleBot3 corriendo):

```bash
ros2 launch trajectory_recorder record.launch.py
```

En otra terminal, mueve el robot con teleop (teclado o joystick). Al
terminar, presiona `Ctrl+C` en la terminal del grabador: la trayectoria se
guarda automáticamente en
`~/turtlebot3_ws/trajectories/trajectory.json`.

También puedes guardar sin detener el nodo:

```bash
ros2 service call /save_trajectory std_srvs/srv/Trigger {}
```

Parámetros útiles:

```bash
ros2 launch trajectory_recorder record.launch.py filename:=vuelta1.json record_rate:=20.0
```

### 2. Reproducir la trayectoria

```bash
ros2 launch trajectory_recorder play.launch.py filename:=vuelta1.json
```

Por defecto `use_pid:=true`, es decir, el reproductor compara la
odometría real contra la odometría grabada en cada instante y corrige la
velocidad lineal/angular para minimizar el error de posición y
orientación. Para reproducir en lazo abierto (sin corrección):

```bash
ros2 launch trajectory_recorder play.launch.py filename:=vuelta1.json use_pid:=false
```

Ajuste de ganancias PID:

```bash
ros2 launch trajectory_recorder play.launch.py \
    filename:=vuelta1.json \
    kp_linear:=0.8 ki_linear:=0.0 kd_linear:=0.05 \
    kp_angular:=1.5 ki_angular:=0.0 kd_angular:=0.1
```

### 3. Optimizar la trayectoria grabada

Una vez grabada una trayectoria, el nodo `optimizer_node` la resume en
**waypoints por segmento** con los campos `segment_id`, `x`, `y`, `yaw` y
`distance` (distancia recorrida en ese segmento):

```bash
ros2 launch trajectory_recorder optimize.launch.py filename:=vuelta1.json
```

Esto genera `trajectory_optimized.json` en la misma carpeta, con una
estructura del tipo:

```json
{
  "waypoints": [
    {"segment_id": 0, "x": 0.0, "y": 0.0, "yaw": 0.0, "distance": 1.15, "yaw_change": 0.0, "motion_type": "straight"},
    {"segment_id": 1, "x": 1.15, "y": 0.0, "yaw": 0.0, "distance": 0.0, "yaw_change": 1.35, "motion_type": "turn"}
  ]
}
```

Parámetros útiles:

```bash
ros2 launch trajectory_recorder optimize.launch.py \
    filename:=vuelta1.json \
    min_segment_distance:=0.1 \
    min_yaw_change:=0.15 \
    simplify_epsilon:=0.03
```

* `min_segment_distance` / `min_yaw_change`: un segmento se descarta solo
  si NO supera ninguno de los dos umbrales (esto evita perder giros en el
  sitio, que tienen `distance ≈ 0` pero `yaw_change` grande).
* `simplify_epsilon`: si es > 0, aplica Douglas-Peucker sobre los puntos
  (x, y) para reducir aún más el número de waypoints conservando la forma
  general de la ruta.
* También puedes re-ejecutar la optimización sin relanzar el nodo:
  `ros2 service call /optimize_trajectory std_srvs/srv/Trigger {}`.

> **Nota:** la salida de `optimizer_node` (waypoints por segmento) tiene un
> formato distinto al de la trayectoria cruda que consume `player.py`
> (muestra a muestra). Es útil para inspeccionar/depurar la ruta o como
> base para un planificador de más alto nivel; si quieres reproducirla
> directamente con el robot, `player.py` necesitaría adaptarse para
> convertir cada waypoint (distancia + giro) en una secuencia de cmd_vel.

## Notas de diseño

* El formato JSON guarda, para cada muestra: tiempo relativo `t`,
  el `cmd_vel` comandado en ese instante y la `odom` (x, y, yaw,
  velocidades) real del robot. Esto permite reproducir tanto en modo
  "feed-forward puro" como con corrección PID usando la odometría
  grabada como referencia punto a punto.
* El PID se aplica sobre dos ejes independientes: error de distancia
  (proyectado sobre el heading actual, para saber si acelerar o frenar)
  y error de yaw (diferencia angular normalizada a [-pi, pi]).
* Los límites `max_linear_correction` / `max_angular_correction` evitan
  que la corrección PID domine sobre el comando grabado.
