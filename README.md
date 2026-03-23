# caddy_ai2_ros2_bicycle_to_ackermann_steering_adapter

## Publicar un valor de posición del steering
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3]}" -r 100

Del TFG - Simulación de vehículos eléctricos ligeros - página 20 y 21 se extreaen las ecuaciones.