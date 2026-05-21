# xACT

xACT is a small Pybricks action framework for LEGO robots. It gives missions a common structure: each robot command returns an action object, actions are composed into sequences or parallel groups, and one runner updates everything until the mission is done.

The project has two robot classes:

| File | Purpose |
|---|---|
| `xAct_robot.py` | Differential-drive robot: left/right drive motors, optional arm motors, color line detection, ultrasonic reads. |
| `xAct_robot_steer.py` | Steering robot: one drive motor and one steering motor. |
| `xAct_action.py` | `Action`, `SequentialAction`, `ParallelAction`, `ConditionalAction`, and `run_actions()`. |
| `xAct_missions.py` | Mission list used by the hub menu example. |
| `xAct_fll_main.py` | Hub-menu runner for multiple missions. |
| `xAct_robomission_main.py` | Single-mission example for the differential robot. |
| `xAct_fe_main.py` | Single-mission example for the steering robot. |
| `CHEATSHEET.md` | Fast API tables, examples, and similar-command differences. |

Both robot classes can work with compatible Pybricks hub objects. The examples use `PrimeHub` for the differential robot and `TechnicHub` for the steering robot, but that is not a hard rule.

## Installation

1. Create a Pybricks project.
2. Add the xACT files you use to that project.
3. Put your robot setup and mission in one of the main files, or create your own main file using the same pattern.
4. Run the main file from Pybricks.

There is no Python package install step. This code is meant to run on the hub with Pybricks.

## Core Ideas

### Actions

Robot commands ending with `_action()` return action objects. They do not run to completion when the list is created. They run when `run_actions()` updates them.

```python
from xAct_action import SequentialAction

def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            robot.drive_to_point_action(x=500, y=0, speed=70),
            robot.turn_to_heading_action(90),
        ])
    ]
```

Normal functions run immediately. For example, `robot.beep()` beeps now. Use `robot.sound_action()` when the beep must happen as a mission step.

### Runner

Use `run_actions()` as the standard mission loop.

```python
from xAct_action import run_actions

odometry = robot.odometry_action()
actions = mission(robot)

robot.beep()
wait(500)
run_actions(robot, actions, odometry)
```

`run_actions()` updates odometry every tick when `odometry` is passed, starts top-level actions through `_safe_update()`, removes finished actions, and prints invalid top-level actions instead of crashing silently.

### Containers

Use action containers to describe mission flow.

```python
SequentialAction(robot, [
    robot.drive_to_point_action(x=500, y=0, speed=80),
    robot.sound_action(),
])
```

```python
ParallelAction(robot, [
    robot.drive_to_point_action(x=800, y=0, speed=70),
    robot.arm_action("L", speed=400, angle=360),
])
```

```python
ConditionalAction(
    robot,
    condition=lambda: robot.ultrasonic_distance(Port.C) < 300,
    true_action=robot.arm_action("L", speed=400, angle=360),
    false_action=robot.drive_to_point_action(x=500, y=0, speed=70),
)
```

`ConditionalAction` checks the condition once when it starts. A branch can be one action or a list of actions; lists are wrapped as `SequentialAction`.

## Units And Conventions

| Topic | Convention |
|---|---|
| Coordinates | `x` and `y` are millimeters. |
| Distances | Millimeters. |
| Headings | Degrees; `0` and `360` are the same direction. |
| Raw `dc()` speed | Clamped to `-100..100` where raw motor power is used. |
| Timed actions | `duration_ms` is milliseconds. |
| Action names | Action-returning commands keep the `_action` suffix. |

## Main File Pattern

```python
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Port
from pybricks.tools import wait
from xAct_action import SequentialAction, run_actions
from xAct_robot import Robot

hub = PrimeHub(front_side=Axis.Y, top_side=Axis.Z)
robot = Robot(
    hub,
    left_motor=Port.B,
    right_motor=Port.F,
    arm_left_port=Port.D,
    arm_right_port=None,
)
odometry = robot.odometry_action()


def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            robot.drive_to_point_action(x=600, y=0, speed=100),
            robot.drive_to_point_action(x=600, y=600, speed=100),
        ])
    ]


actions = mission(robot)

robot.beep()
wait(500)

run_actions(robot, actions, odometry)
```

## Robot Notes

The constructors wait up to 5 seconds for the IMU. If the IMU is still not ready, they raise `RuntimeError("IMU is not ready.")`.

When `ultrasonic_distance(sensor)` receives a port, the robot creates the ultrasonic sensor once and caches it. Passing an existing sensor object still works.

Coordinate movement depends on odometry. If you use `drive_to_point_action()`, `turn_to_heading_action()`, or steering movement, pass `odometry` to `run_actions()`.

For full function tables and quick examples, use `CHEATSHEET.md`.
