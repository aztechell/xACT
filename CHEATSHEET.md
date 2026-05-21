# xACT Cheatsheet

Fast reference for xACT functions, action containers, common examples, and similar-command differences.

## Standard Main Template

```python
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Port
from pybricks.tools import wait
from xAct_action import SequentialAction, run_actions
from xAct_robot import Robot

hub = PrimeHub(front_side=Axis.Y, top_side=Axis.Z)
robot = Robot(hub, left_motor=Port.B, right_motor=Port.F)
odometry = robot.odometry_action()


def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            robot.drive_to_point_action(x=500, y=0, speed=80),
        ])
    ]


actions = mission(robot)
robot.beep()
wait(500)
run_actions(robot, actions, odometry)
```

## Action Containers

| Function | Purpose |
|---|---|
| `Action(robot)` | Base class for custom actions. Override `update()` and optionally `on_start()`. |
| `SequentialAction(robot, actions)` | Run actions one after another. |
| `ParallelAction(robot, actions)` | Run actions at the same time until all finish. |
| `ConditionalAction(robot, condition, true_action, false_action=None)` | Check condition once, then run one selected branch. |
| `run_actions(robot, actions, odometry=None)` | Run top-level actions and update odometry every tick. |

## Differential Robot Actions

Use these with `from xAct_robot import Robot`.

| Function | Purpose |
|---|---|
| `robot.odometry_action()` | Update `X`, `Y`, `heading`, and `velocity`. |
| `robot.reset_odometry_action(x=None, y=None, heading=None)` | Set position and/or heading. |
| `robot.drive_to_point_action(x=None, y=None, speed=90)` | Drive to a field coordinate in millimeters. |
| `robot.straight_action(distance, speed=90)` | Drive a distance from the current pose. |
| `robot.turn_to_heading_action(target_heading, max_speed=50, tolerance=3, timeout_ms=5000)` | Turn to an absolute heading. |
| `robot.one_wheel_turn_action(wheel, angle, max_speed=50, timeout_ms=5000)` | Turn around one wheel until heading changes by `angle`. |
| `robot.straight_to_line_action(sensor, threshold, speed=50, stop=Stop.HOLD, stop_when="less")` | Drive straight until color reflection crosses a threshold. |
| `robot.arm_action(arm, speed, angle, stop=Stop.HOLD, waiting=True)` | Move an arm motor by angle. |
| `robot.single_wheel_action(wheel, speed, angle, stop=Stop.HOLD, waiting=True)` | Rotate one drive wheel motor by angle. |
| `robot.wall_bump_action(wall, speed, duration_ms=0, stop=Stop.BRAKE)` | Drive into a wall for a fixed time, stop, then roughly reset pose. |
| `robot.arm_dc_action(arm, speed, duration_ms=0, stop=Stop.HOLD)` | Run an arm motor with `dc()` for a fixed time, then stop it. |
| `robot.sound_action()` | Beep as an action step. |
| `robot.print_pose_action(mode="drive", speed=None)` | Print current pose once. |
| `robot.print_ultrasonic_distance_action(sensor, label="Distance")` | Print one ultrasonic distance reading. |
| `robot.print_pose_on_button_action(button=Button.CENTER, one_shot=True, beep=True, mode="drive", speed=None)` | Print pose when a hub button is pressed. |
| `robot.wait_action(duration_ms)` | Non-blocking wait. |

## Differential Robot Normal Functions

| Function | Purpose |
|---|---|
| `robot.ultrasonic_distance(sensor)` | Return ultrasonic distance in millimeters. Passing a port creates and caches the sensor. |
| `robot.beep()` | Beep immediately. |

## Steering Robot Actions

Use these with `from xAct_robot_steer import Robot`.

| Function | Purpose |
|---|---|
| `robot.odometry_action()` | Update `X`, `Y`, `heading`, `heading_abs`, and `velocity`. |
| `robot.reset_odometry_action(x=None, y=None, heading=None)` | Set position and/or heading. |
| `robot.drive_distance_action(distance_mm, speed=60, heading=None, kp=2.0, kd=0.5, ki=0.0, max_steer_deg=None, min_speed=30, stop=Stop.HOLD, timeout_ms=None)` | Drive a distance while holding a heading. |
| `robot.drive_to_point_action(x=None, y=None, speed=60, kp=None, kd=None, ki=None, max_steer_deg=None, min_speed=None, stop_dist_mm=None, i_limit=None, pose_offset_mm=None, max_accel_dist_mm=None, max_decel_dist_mm=None, stop=Stop.COAST)` | Drive to a field coordinate with steering. |
| `robot.turn_to_heading_action(target_heading, speed=30, max_steer_deg=None, kp=2.0, tolerance_deg=2.0, stop=Stop.HOLD, timeout_ms=5000)` | Turn toward a heading by driving and steering. |
| `robot.steer_action(target_deg, wait=False)` | Move steering motor to an angle. |
| `robot.drive_dc_action(speed, duration_ms=None, stop=Stop.BRAKE)` | Run drive motor with `dc()`; `None` duration runs forever. |
| `robot.stop_action(stop=Stop.BRAKE, center_steer=True)` | Stop drive and optionally center steering. |
| `robot.sound_action()` | Beep as an action step. |
| `robot.print_pose_action()` | Print current pose once. |
| `robot.print_ultrasonic_distance_action(sensor, label="Distance")` | Print one ultrasonic distance reading. |
| `robot.wait_action(duration_ms)` | Non-blocking wait. |

## Steering Robot Normal Functions

| Function | Purpose |
|---|---|
| `robot.set_drive_to_point_tuning(...)` | Change default tuning for steering `drive_to_point_action()`. |
| `robot.ultrasonic_distance(sensor)` | Return ultrasonic distance in millimeters. Passing a port creates and caches the sensor. |
| `robot.beep()` | Beep immediately; ignored if the hub has no speaker. |

## Examples

### Sequential

```python
SequentialAction(robot, [
    robot.drive_to_point_action(x=500, y=0, speed=80),
    robot.turn_to_heading_action(90),
    robot.sound_action(),
])
```

### Parallel

```python
ParallelAction(robot, [
    robot.drive_to_point_action(x=800, y=0, speed=70),
    robot.arm_action("L", speed=400, angle=360),
])
```

### Conditional

```python
ConditionalAction(
    robot,
    condition=lambda: robot.ultrasonic_distance(Port.C) < 300,
    true_action=[
        robot.turn_to_heading_action(170),
        robot.arm_action("L", speed=400, angle=360),
    ],
    false_action=[
        robot.turn_to_heading_action(190),
        robot.arm_action("L", speed=400, angle=360),
    ],
)
```

### Menu Variant

```python
choice = hub_menu("1", "2", "3")

if choice == "1":
    route = [
        robot.drive_to_point_action(x=40, y=-470, speed=100),
        robot.drive_to_point_action(x=40, y=-1060, speed=100),
    ]
elif choice == "2":
    route = [
        robot.drive_to_point_action(x=380, y=-470, speed=100),
        robot.drive_to_point_action(x=380, y=-1060, speed=100),
    ]
else:
    route = [
        robot.drive_to_point_action(x=40, y=-470, speed=100),
        robot.drive_to_point_action(x=210, y=-760, speed=100),
        robot.drive_to_point_action(x=380, y=-1060, speed=100),
    ]

return [
    SequentialAction(robot, [
        robot.reset_odometry_action(x=0, y=0, heading=0),
        *route,
        robot.drive_to_point_action(x=210, y=-1250, speed=100),
    ])
]
```

## Similar Functions

| Functions | Difference |
|---|---|
| `drive_to_point_action(x, y, speed)` vs `straight_action(distance, speed)` | `drive_to_point_action()` goes to a coordinate. `straight_action()` drives a relative distance from the current pose. |
| `single_wheel_action(wheel, speed, angle)` vs `one_wheel_turn_action(wheel, angle, max_speed)` | `single_wheel_action()` controls wheel motor degrees. `one_wheel_turn_action()` controls robot heading. |
| `robot.beep()` vs `robot.sound_action()` | `beep()` runs immediately. `sound_action()` is used inside action sequences. |
| `ultrasonic_distance(sensor)` vs `print_ultrasonic_distance_action(sensor)` | `ultrasonic_distance()` returns a number now. `print_ultrasonic_distance_action()` prints as an action step. |
| `arm_action(arm, speed, angle)` vs `arm_dc_action(arm, speed, duration_ms)` | `arm_action()` runs by motor degrees. `arm_dc_action()` runs raw power for time. |
| `drive_distance_action(distance_mm, ...)` vs steering `drive_to_point_action(x, y, ...)` | `drive_distance_action()` drives a relative wheel distance. Steering `drive_to_point_action()` drives to a coordinate. |

## Notes

| Topic | Rule |
|---|---|
| Coordinates and distances | Millimeters. |
| `duration_ms` | Milliseconds. |
| Heading | Degrees; `0` and `360` are the same direction. |
| Raw `dc()` speed | Clamped to `-100..100`. |
| Odometry | Pass `odometry` to `run_actions()` when using coordinate or heading actions. |
| Ultrasonic sensors | Passing a port creates and caches the sensor. Passing an existing sensor object uses it directly. |
| IMU startup | Robot constructors wait up to 5 seconds, then raise `RuntimeError("IMU is not ready.")`. |
