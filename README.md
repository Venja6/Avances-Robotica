# Avances-Robotica
Repositorio para no perder avances y compartir el estado del trabajo

# Nuevos nodos 
Line follower/follower (Importado de https://github.com/gabrielnhn/ros2-line-follower)
trajectory recorder (Creado para grabar movimientos, elegir y reproducir movimientos)

# Comandos
Con el robot y su càmara funcionando, los siguientes comandos son usados en orden para comenzar con el funcionamiento de los nodos para grabar y reproducir trayectorias:

1. Bringups del robot, robot lauch y camera launch 
2. ros2 run follower follower_node 	(Toma los datos de la càmara del robot y muestra la linea que reconoce usando su contorno y las diferencias en el color)
3. ros2 launch trajectory_recorder record.launch.py	(Empieza a grabar el recorrido que realiza hasta que se detenga con ctrl+C)
4. ros2 service call /start_follower std_srvs/srv/Empty 	(Empieza a mover al robot a lo largo de la linea, tiene un temporizador que lo detiene cuando no encuantra nada. Tambien se puede llamar a .../stop_follower... para forzar a que se detenga)
5. ros2 run trajectory_recorder optimizer 	(Usando datos de tiempo y velocidad, elige el mejor camino disponible si se grabò màs de uno, reproducièndolo en su totalidad)

# Integrantes
1. Juan Silva Fuentes
2. Benjamìn Dìaz Ulloa
3. Sebastiàn Garcias Cabrera
