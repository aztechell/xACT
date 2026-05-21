# xACT

xACT is a small action-based robotics toolkit for Pybricks. It lets you build robot missions from reusable actions that can run sequentially or in parallel without turning the whole program into blocking control flow.

The project currently contains two robot implementations:

- `xAct_robot.py` - a differential-drive robot with left and right drive motors, optional arm motors, and optional color-sensor line detection.
- `xAct_robot_steer.py` - a car-like robot with one drive motor and one steering motor.

Both robot classes can work with any compatible Pybricks hub object. The current examples use `PrimeHub` for the differential-drive robot and `TechnicHub` for the steering robot, but that is a project choice, not a hard framework rule.

## Files

| File | Purpose |
| --- | --- |
| `xAct_action.py` | Base `Action`, `SequentialAction`, `ParallelAction`, and `ConditionalAction`. |
| `xAct_robot.py` | Differential-drive robot actions. |
| `xAct_robot_steer.py` | Single-drive + steering robot actions. |
| `xAct_missions.py` | Mission list used by `xAct_fll_main.py`. |
| `xAct_fll_main.py` | Hub-menu runner for multiple missions. |
| `xAct_robomission_main.py` | Single-mission example for `xAct_robot.py`. |
| `xAct_fe_main.py` | Single-mission example for `xAct_robot_steer.py`. |

## Action Model

Every robot command returns an action object. An action is updated repeatedly until it returns `True`.

```python
while actions:
    odometry.update()
    for action in actions[:]:
        if action.update():
            actions.remove(action)
```

Most missions return a list containing one `SequentialAction`:

```python
from xAct_action import SequentialAction, ParallelAction

def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(0, 0, 0),
            robot.drive_to_point_action(_X=500, _Y=0, speed=70),
            ParallelAction(robot, [
                robot.drive_to_point_action(_X=800, _Y=0, speed=70),
                robot.arm_action("L", 50, 90),
            ]),
            robot.turn_to_heading_action(0),
        ])
    ]
```

## `xAct_action.py`

### `Action`

Base class for custom actions.

- `on_start()` runs once before the first update when the action is managed by `SequentialAction` or `ParallelAction`.
- `update()` contains the action logic. Return `True` when the action is complete.
- `wait(duration_ms)` is a non-blocking timer helper for use inside `update()`.

```python
from xAct_action import Action

class PrintOnce(Action):
    def update(self):
        print("done")
        return True
```

### `SequentialAction(robot, actions)`

Runs actions one after another. The next action starts only after the current one finishes.

```python
SequentialAction(robot, [
    robot.reset_odometry_action(0, 0, 0),
    robot.straight_action(500, speed=80),
    robot.turn_to_heading_action(90),
])
```

### `ParallelAction(robot, actions)`

Runs several actions at the same time. The group finishes when all child actions finish.

```python
ParallelAction(robot, [
    robot.straight_action(300, speed=70),
    robot.arm_action("L", speed=60, angle=120),
])
```

### `ConditionalAction(robot, condition, true_action, false_action=None)`

Checks a condition when the action starts and then runs one selected branch. `condition` can be a function or a boolean value.

```python
from pybricks.parameters import Port
from xAct_action import ConditionalAction

ConditionalAction(
    robot,
    condition=lambda: robot.ultrasonic_distance(Port.C) < 300,
    true_action=robot.arm_action("L", speed=500, angle=90),
    false_action=robot.drive_to_point_action(_X=500, _Y=0, speed=70),
)
```

Each branch can also be a list of actions. Lists are automatically wrapped in `SequentialAction`.

```python
ConditionalAction(
    robot,
    condition=lambda: robot.ultrasonic_distance(Port.C) < 300,
    true_action=[
        robot.arm_action("L", speed=500, angle=90),
        robot.wait_action(300),
        robot.arm_action("L", speed=500, angle=-90),
    ],
    false_action=[
        robot.drive_to_point_action(_X=500, _Y=0, speed=70),
        robot.print_pose_action(),
    ],
)
```

If `false_action` is omitted and the condition is false, the `ConditionalAction` finishes immediately.

## Differential-Drive Robot

Use `xAct_robot.py` when the robot drives with independent left and right motors. The current implementation stores `X` and `Y` in millimeters.

```python
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Port
from xAct_robot import Robot

hub = PrimeHub(front_side=Axis.Y, top_side=Axis.Z)
robot = Robot(
    hub,
    left_motor=Port.E,
    right_motor=Port.A,
    arm_left_port=Port.F,
    arm_right_port=Port.B,
)
odometry = robot.odometry_action()
```

Constructor arguments:

- `hub` - compatible Pybricks hub object.
- `left_motor`, `right_motor` - drive motor ports.
- `arm_left_port`, `arm_right_port` - optional arm motor ports.
- `wheel_diameter_mm` - wheel diameter.
- `track_width_mm` - distance between left and right wheels.
- `left_dir`, `right_dir` - motor directions.

The constructor waits up to 5 seconds for the IMU to become ready. If the IMU is still not ready, it raises `RuntimeError`.

### Movement

`odometry_action()`

Updates `robot.X`, `robot.Y`, `robot.heading`, and `robot.velocity`. Call it continuously in the main loop when using coordinate-based movement.

```python
odometry = robot.odometry_action()

while actions:
    odometry.update()
    ...
```

`drive_to_point_action(_X=None, _Y=None, speed=90)`

Drives to a coordinate. If `_X` or `_Y` is `None`, that coordinate stays at the current value.

```python
robot.drive_to_point_action(_X=600, _Y=300, speed=80)
robot.drive_to_point_action(_X=None, _Y=800, speed=70)
```

`straight_action(_distance, speed=90)`

Drives straight from the current pose by `_distance`.

```python
robot.straight_action(400, speed=80)
robot.straight_action(200, speed=-60)
```

`turn_to_heading_action(target_heading, max_speed=50, tolerance=3, timeout_ms=5000)`

Turns the robot to an absolute heading using `robot.heading`, so `odometry.update()` must keep running in the main loop. If it cannot reach the target before `timeout_ms`, it stops and finishes instead of blocking forever. Use `timeout_ms=None` to disable the timeout.

```python
robot.turn_to_heading_action(90)
robot.turn_to_heading_action(0, max_speed=40)
robot.turn_to_heading_action(170, tolerance=5, timeout_ms=7000)
```

`one_wheel_turn_action(Wheel_Name, angle, max_speed=50, timeout_ms=5000)`

Turns around one wheel. `Wheel_Name` is `"L"` or `"R"`. If the turn cannot finish before `timeout_ms`, it stops and finishes instead of blocking forever. Use `timeout_ms=None` to disable the timeout.

```python
robot.one_wheel_turn_action("L", 90, max_speed=60)
```

`straight_to_line_action(sensor, threshold, speed=50, stop=Stop.HOLD, stop_when="less")`

Drives straight until a color sensor reflection threshold is reached. `sensor` can be a port or an existing `ColorSensor`.

```python
from pybricks.parameters import Port

robot.straight_to_line_action(Port.C, threshold=30, speed=50)
robot.straight_to_line_action(Port.C, threshold=70, speed=40, stop_when="greater")
```

### Motors and Attachments

`arm_action(Arm_Name, speed, angle, stop=Stop.HOLD, waiting=True)`

Runs an arm motor by angle. `Arm_Name` is `"L"` or `"R"`. With `waiting=True`, the action stays active until the motor finishes, but the main action loop keeps running. With `waiting=False`, the motor is started and the action finishes immediately.

```python
robot.arm_action("L", speed=60, angle=120)
```

`arm_dc(Arm_Name, speed, waitTime=0, stop=Stop.HOLD)`

Runs an arm motor with `dc(speed)` for `waitTime` seconds, then stops it with the selected stop mode.

```python
robot.arm_dc("R", speed=-50, waitTime=0.5)
```

`single_wheel_action(Wheel_Name, speed, angle, stop=Stop.HOLD, waiting=True)`

Runs one drive wheel by angle. `Wheel_Name` is `"L"` or `"R"`. With `waiting=True`, the action stays active until the motor finishes, but the main action loop keeps running. With `waiting=False`, the motor is started and the action finishes immediately.

```python
robot.single_wheel_action("R", speed=70, angle=180)
```

`wall_bump(WallName, speed, waitTime=0, stop=Stop.BRAKE)`

Drives into a wall for a fixed time, stops the drive motors, and then roughly resets position. If `WallName == "U"`, it sets `X = 1052` and heading `180`; otherwise it sets `X = 0` and heading `0`.

```python
robot.wall_bump("U", speed=40, waitTime=1)
```

### Utility Actions

`reset_odometry_action(_X=None, _Y=None, heading=None)`

Sets position and heading.

```python
robot.reset_odometry_action(0, 0, 0)
```

`print_pose_action(mode="drive", speed=None)`

Prints the current pose. Default mode prints a reusable `drive_to_point_action(...)` line. Use `mode="xyh"` for plain coordinates.

```python
robot.print_pose_action()
robot.print_pose_action(mode="xyh")
```

`ultrasonic_distance(sensor)`

Reads an ultrasonic sensor and returns distance in millimeters. `sensor` can be a port or an already created `UltrasonicSensor`. When a port is passed, the robot caches the sensor object and reuses it.

```python
from pybricks.parameters import Port

distance = robot.ultrasonic_distance(Port.C)
print(distance)
```

`print_ultrasonic_distance_action(sensor, label="Distance")`

Prints one ultrasonic distance reading as an action.

```python
robot.print_ultrasonic_distance_action(Port.C)
```

`print_pose_on_button_action(button=Button.CENTER, one_shot=True, beep=True, mode="drive", speed=None)`

Prints pose when a hub button is pressed.

```python
from pybricks.parameters import Button

robot.print_pose_on_button_action(Button.CENTER)
```

`wait_action(duration_ms)`

Non-blocking delay.

```python
robot.wait_action(500)
```

`sound_action()`

Beeps once.

```python
robot.sound_action()
```

## Steering Robot

Use `xAct_robot_steer.py` when the robot has one drive motor and one steering motor. The current implementation stores `X` and `Y` in millimeters and keeps both wrapped heading (`heading`) and absolute unwrapped heading (`heading_abs`).

```python
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction
from xAct_robot_steer import Robot

hub = TechnicHub()
robot = Robot(
    hub,
    drive_port=Port.A,
    steer_port=Port.B,
    steer_dir=Direction.COUNTERCLOCKWISE,
)
odometry = robot.odometry_action()
```

Constructor arguments:

- `hub` - compatible Pybricks hub object.
- `drive_port`, `steer_port` - drive and steering motor ports.
- `drive_dir`, `steer_dir` - motor directions.
- `wheel_diameter_mm` - wheel diameter.
- `drive_gear_ratio` - drive gearing ratio.
- `steer_speed` - steering motor speed.
- `max_steer_deg` - steering angle limit.
- `steer_center` - steering center offset.
- `steer_calibrate_offset` - startup steering calibration target, or `None`.
- `steer_inverted` - reverses steering sign.
- `heading_zero` - initial absolute heading.

The constructor waits up to 5 seconds for the IMU to become ready. If the IMU is still not ready, it raises `RuntimeError`.

### Tuning

`set_drive_to_point_tuning(kp=None, ki=None, kd=None, max_steer_deg=None, min_speed=None, stop_dist_mm=None, i_limit=None, pose_offset_mm=None, max_accel_dist_mm=None, max_decel_dist_mm=None)`

Changes default tuning for `drive_to_point_action()`.

```python
robot.set_drive_to_point_tuning(
    kp=2.5,
    kd=0.7,
    min_speed=25,
    stop_dist_mm=20,
)
```

### Movement

`odometry_action()`

Updates `X`, `Y`, `heading`, `heading_abs`, and `velocity`. Call it continuously when using `drive_to_point_action()`.

`drive_distance_action(distance_mm, speed=60, heading=None, kp=2.0, kd=0.5, ki=0.0, max_steer_deg=None, min_speed=30, stop=Stop.HOLD, timeout_ms=None)`

Drives a distance while holding a heading with steering. If `heading=None`, it holds the current heading.

```python
robot.drive_distance_action(2000, speed=80, heading=0)
robot.drive_distance_action(-500, speed=-50, heading=180)
```

`drive_to_point_action(_X=None, _Y=None, speed=60, kp=None, kd=None, ki=None, max_steer_deg=None, min_speed=None, stop_dist_mm=None, i_limit=None, pose_offset_mm=None, max_accel_dist_mm=None, max_decel_dist_mm=None, stop=Stop.COAST)`

Drives to a coordinate in millimeters. Requires odometry updates.

```python
robot.drive_to_point_action(_X=1000, _Y=0, speed=70)
robot.drive_to_point_action(_X=1000, _Y=500, speed=60, stop_dist_mm=30)
```

`turn_to_heading_action(target_heading, speed=30, max_steer_deg=None, kp=2.0, tolerance_deg=2.0, stop=Stop.HOLD, timeout_ms=5000)`

Turns toward a heading by driving and steering. If it cannot reach the target before `timeout_ms`, it stops and finishes instead of blocking forever. Use `timeout_ms=None` to disable the timeout.

```python
robot.turn_to_heading_action(90, speed=25)
```

`steer_action(target_deg, wait=False)`

Turns the steering motor to a target angle.

```python
robot.steer_action(30)
robot.steer_action(0, wait=True)
```

`drive_dc_action(speed, duration_ms=None, stop=Stop.BRAKE)`

Runs the drive motor with `dc(speed)`. If `duration_ms=None`, the action does not finish by itself.

```python
robot.drive_dc_action(50, duration_ms=1000)
```

`stop_action(stop=Stop.BRAKE, center_steer=True)`

Stops the drive motor and optionally centers steering.

```python
robot.stop_action(center_steer=True)
```

### Utility Actions

`reset_odometry_action(_X=None, _Y=None, heading=None)`

Sets position and absolute heading.

```python
robot.reset_odometry_action(0, 0, 0)
```

`print_pose_action()`

Prints position in millimeters and absolute heading.

```python
robot.print_pose_action()
```

`ultrasonic_distance(sensor)`

Reads an ultrasonic sensor and returns distance in millimeters. `sensor` can be a port or an already created `UltrasonicSensor`. When a port is passed, the robot caches the sensor object and reuses it.

```python
from pybricks.parameters import Port

distance = robot.ultrasonic_distance(Port.C)
print(distance)
```

`print_ultrasonic_distance_action(sensor, label="Distance")`

Prints one ultrasonic distance reading as an action.

```python
robot.print_ultrasonic_distance_action(Port.C)
```

`wait_action(duration_ms)` and `sound_action()`

Work like the differential-drive versions.

## Mission Runner Examples

### Menu Runner

`xAct_fll_main.py` shows a hub menu and runs a mission from `mission_list`.

```python
from xAct_missions import mission_list

menu_items = tuple(str(i + 1) for i in range(len(mission_list)))
choice = hub_menu(*menu_items)
mission_func = mission_list[int(choice) - 1]
actions = mission_func(robot)
```

To add a mission:

1. Create a function in `xAct_missions.py`.
2. Return a list of actions.
3. Add the function to `mission_list`.

```python
def mission3(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(0, 0, 0),
            robot.straight_action(800, 100),
        ])
    ]

mission_list = [mission1, mission2, mission3]
```

### Steering Robot Single Mission

`xAct_fe_main.py` is a minimal steering-robot example:

```python
def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(0, 0, 0),
            robot.drive_distance_action(2000, speed=100, heading=0),
            robot.print_pose_action(),
        ])
    ]
```

## Helper Functions

`speed_control(ratio, max_speed, min_speed=30)`

Applies acceleration and deceleration based on progress from `0` to `1`.

`clamp(value, min_val, max_val)`

Limits a value to a range.

`angle_wrap(value)`

Wraps an angle to `-180..180`.

`HeadingTracker`

Available in `xAct_robot_steer.py`. Tracks absolute heading so turns can go beyond `180` or below `-180`.

## Practical Notes

- Keep `odometry.update()` in the main loop when using coordinate movement.
- Put robot actions inside `SequentialAction`, `ParallelAction`, or `ConditionalAction` when you want `on_start()` to run correctly.
- Both robot implementations use millimeters for coordinates and distances.
- Speed values passed to `dc()` and most `speed` arguments are usually in the `-100..100` range.
- If a required motor is not configured, the relevant action raises `RuntimeError`.
