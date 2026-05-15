# bicycle_to_ackermann_steering_adapter

A `ros2_control` chainable controller that converts a **bicycle-model center steering angle** into individual **left and right Ackermann steering angles**.

The bicycle model (used by `bicycle_steering_controller`) represents front steering as a single virtual wheel at the vehicle centerline. Real Ackermann vehicles have two physically separated steering wheels that must turn at different angles to avoid tire slip. This controller performs that geometric conversion every control cycle.

The Ackermann steering equations are taken from the TFG *"Simulación de vehículos eléctricos ligeros"*, pages 20–21.

## Kinematics

Given the center steering angle **δ** and the vehicle geometry:

```
cot(δ) = cos(|δ|) / sin(|δ|)

δ_outer = atan2(1,  cot(δ) + track_width / (2 · wheelbase))
δ_inner = atan2(1,  cot(δ) - track_width / (2 · wheelbase))
```

Sign assignment follows [REP-103](https://www.ros.org/reps/rep-0103.html): a positive δ means turning left, so the left wheel is inner and the right wheel is outer.

State feedback path (for upstream odometry): the controller also reads back the real joint positions and converts them to an equivalent center steering angle exposed as a chained state interface.

## Controller chain

```
[bicycle_steering_controller]
        │ reference interface: center_steering_joint/position
        ▼
[bicycle_to_ackermann_steering_adapter]   ← this package
        │ command interfaces: right_wheel_steering_joint/position
        │                     left_wheel_steering_joint/position
        ▼
[hardware / ros2_control joints]
```

## Parameters

| Parameter | Type | Description |
|---|---|---|
| `wheelbase` | `double` (> 0) | Distance between front and rear axles (m). See [Wikipedia: Wheelbase](https://en.wikipedia.org/wiki/Wheelbase). |
| `track_width` | `double` (> 0) | Distance between the two front steering wheels (m). |
| `max_steer_angle` | `double` (> 0) | Maximum center steering angle (rad). Commands are clamped to `[-max_steer_angle, max_steer_angle]`. |
| `input_steering_name` | `string` | Name of the chained input interface (e.g. `center_steering_joint`). |
| `output_steering_names` | `string[]` (size = 2) | Hardware joint names in order: `[right_joint, left_joint]`. |

### Example configuration

```yaml
bicycle_to_ackermann_steering_adapter:
  ros__parameters:
    type: 'bicycle_to_ackermann_steering_adapter/BicycleToAckermannSteeringAdapter'
    wheelbase: 1.7
    track_width: 1.0
    max_steer_angle: 0.4
    input_steering_name: 'center_steering_joint'
    output_steering_names: ['right_wheel_steering_joint', 'left_wheel_steering_joint']
```

## Build

```bash
cd <ros2_ws>
colcon build --packages-select bicycle_to_ackermann_steering_adapter
source install/setup.bash
```
