# xACT Cheatsheet

## Action Lists

Robot commands that end with `_action()` return action objects. Put these inside `SequentialAction`, `ParallelAction`, or `ConditionalAction`.

```python
SequentialAction(robot, [
    robot.drive_to_point_action(_X=500, _Y=0, speed=80),
    robot.sound_action(),
])
```

Do not put normal functions inside an action list.

```python
# Wrong: beep() runs immediately while the list is created.
SequentialAction(robot, [
    robot.drive_to_point_action(_X=500, _Y=0, speed=80),
    robot.beep(),
])

# Correct: sound_action() runs at this step in the sequence.
SequentialAction(robot, [
    robot.drive_to_point_action(_X=500, _Y=0, speed=80),
    robot.sound_action(),
])
```

## Common Action Containers

| Action | Purpose |
|---|---|
| `SequentialAction(robot, actions)` | Run actions one after another. |
| `ParallelAction(robot, actions)` | Run actions at the same time until all finish. |
| `ConditionalAction(robot, condition, true_action, false_action=None)` | Check condition once, then run one selected branch. |

## Differential Robot: `xAct_robot.py`

Use this robot when the drive base has independent left and right motors.

| Function | Type | Purpose |
|---|---|---|
| `robot.odometry_action()` | action | Update `X`, `Y`, `heading`, and `velocity`. Keep this running in the main loop. |
| `robot.drive_to_point_action(_X=None, _Y=None, speed=90)` | action | Drive to a field coordinate in millimeters. |
| `robot.straight_action(_distance, speed=90)` | action | Drive straight by a distance from the current pose. |
| `robot.turn_to_heading_action(target_heading, max_speed=50, tolerance=3, timeout_ms=5000)` | action | Turn to an absolute heading. |
| `robot.one_wheel_turn_action(Wheel_Name, angle, max_speed=50, timeout_ms=5000)` | action | Turn around one wheel until heading changes by `angle`. |
| `robot.arm_action(Arm_Name, speed, angle, stop=Stop.HOLD, waiting=True)` | action | Move an arm motor by angle. |
| `robot.single_wheel_action(Wheel_Name, speed, angle, stop=Stop.HOLD, waiting=True)` | action | Rotate one drive wheel motor by angle. |
| `robot.wall_bump(WallName, speed, waitTime=0, stop=Stop.BRAKE)` | action | Drive into a wall for a fixed time, stop, then roughly reset pose. |
| `robot.arm_dc(Arm_Name, speed, waitTime=0, stop=Stop.HOLD)` | action | Run an arm motor with `dc()` for a fixed time, then stop it. |
| `robot.sound_action()` | action | Beep as an action step. |
| `robot.print_pose_action(mode="drive", speed=None)` | action | Print the current pose once. |
| `robot.print_ultrasonic_distance_action(sensor, label="Distance")` | action | Print one ultrasonic distance reading. |
| `robot.print_pose_on_button_action(button=Button.CENTER, one_shot=True, beep=True, mode="drive", speed=None)` | action | Print pose when a hub button is pressed. |
| `robot.wait_action(duration_ms)` | action | Non-blocking wait. |
| `robot.straight_to_line_action(sensor, threshold, speed=50, stop=Stop.HOLD, stop_when="less")` | action | Drive straight until color sensor reflection reaches a threshold. |
| `robot.reset_odometry_action(_X=None, _Y=None, heading=None)` | action | Set `X`, `Y`, and/or heading. |
| `robot.ultrasonic_distance(sensor)` | normal function | Return ultrasonic distance in millimeters. |
| `robot.beep()` | normal function | Beep immediately. |

## Steering Robot: `xAct_robot_steer.py`

Use this robot when the drive base has one drive motor and one steering motor.

| Function | Type | Purpose |
|---|---|---|
| `robot.set_drive_to_point_tuning(...)` | normal function | Change default tuning for `drive_to_point_action()`. |
| `robot.odometry_action()` | action | Update `X`, `Y`, `heading`, `heading_abs`, and `velocity`. |
| `robot.drive_distance_action(distance_mm, speed=60, heading=None, ...)` | action | Drive a distance while holding a heading. |
| `robot.drive_to_point_action(_X=None, _Y=None, speed=60, ...)` | action | Drive to a field coordinate with steering. |
| `robot.turn_to_heading_action(target_heading, speed=30, ..., timeout_ms=5000)` | action | Turn toward a heading by driving and steering. |
| `robot.steer_action(target_deg, wait=False)` | action | Move the steering motor to an angle. |
| `robot.drive_dc_action(speed, duration_ms=None, stop=Stop.BRAKE)` | action | Run the drive motor with `dc()`. |
| `robot.stop_action(stop=Stop.BRAKE, center_steer=True)` | action | Stop drive and optionally center steering. |
| `robot.wait_action(duration_ms)` | action | Non-blocking wait. |
| `robot.sound_action()` | action | Beep as an action step. |
| `robot.print_pose_action()` | action | Print current pose once. |
| `robot.print_ultrasonic_distance_action(sensor, label="Distance")` | action | Print one ultrasonic distance reading. |
| `robot.reset_odometry_action(_X=None, _Y=None, heading=None)` | action | Set `X`, `Y`, and/or heading. |
| `robot.ultrasonic_distance(sensor)` | normal function | Return ultrasonic distance in millimeters. |
| `robot.beep()` | normal function | Beep immediately. |

## Similar Functions

| Functions | Difference |
|---|---|
| `drive_to_point_action(_X, _Y, speed)` vs `straight_action(_distance, speed)` | `drive_to_point_action()` goes to a coordinate. `straight_action()` drives a distance from the current pose. |
| `single_wheel_action(Wheel_Name, speed, angle)` vs `one_wheel_turn_action(Wheel_Name, angle, max_speed)` | `single_wheel_action()` controls wheel motor degrees. `one_wheel_turn_action()` controls robot heading. |
| `robot.beep()` vs `robot.sound_action()` | `beep()` runs immediately. `sound_action()` is used inside action sequences. |
| `ultrasonic_distance(sensor)` vs `print_ultrasonic_distance_action(sensor)` | `ultrasonic_distance()` returns a number now. `print_ultrasonic_distance_action()` prints as an action step. |

## Notes

| Topic | Rule |
|---|---|
| Units | Coordinates and distances are in millimeters. |
| Speed | `dc()` speed values are clamped to `-100..100` where raw `dc()` is used. |
| Heading | Heading is in degrees. `0` and `360` are the same direction. |
| Odometry | Keep `odometry.update()` in the main loop when using coordinate or heading actions. |
| Ultrasonic sensors | Passing a port creates and caches the sensor. Passing an existing sensor object uses it directly. |
