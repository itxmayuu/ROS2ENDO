ROS2ENDO - codigos recuperados y reorganizados

Workspace previsto:
  ~/prueba_ws

Paquete ROS 2:
  ros2endo

Archivos principales:
  prueba_ws/src/ros2endo/ros2endo/esp32_bridge.py
  prueba_ws/src/ros2endo/ros2endo/lidar_node.py
  prueba_ws/src/ros2endo/ros2endo/wall_follower.py
  prueba_ws/src/ros2endo/ros2endo/wall_follower_estable.py
  prueba_ws/src/ros2endo/ros2endo/experiment_logger.py

Configuracion del paquete:
  prueba_ws/src/ros2endo/setup.py
  prueba_ws/src/ros2endo/setup.cfg
  prueba_ws/src/ros2endo/package.xml

Analisis de datos:
  ROS2ENDO_DATA/graficar_distancia.py
  ROS2ENDO_DATA/graficar_todo.py

Firmware ESP32:
  ESP32/src/main.cpp

Comandos de compilacion:
  cd ~/prueba_ws
  colcon build --packages-select ros2endo
  source install/setup.bash

Nodos:
  ros2 run ros2endo esp32_bridge
  ros2 run ros2endo lidar_node
  ros2 run ros2endo wall_follower
  ros2 run ros2endo experiment_logger

Baseline estable alternativo:
  ros2 run ros2endo wall_follower_estable

Importante:
- No ejecutar wall_follower y wall_follower_estable al mismo tiempo porque ambos publican /velocidad_deseada.
- El lidar_node entregado tiene apertura total de 40 grados y setpoint de seguimiento de 0.50 m en los controladores.
- wall_follower.py corresponde a la version fuzzy-PD.
- wall_follower_estable.py conserva la version fuzzy anterior con salidas suavizadas.
