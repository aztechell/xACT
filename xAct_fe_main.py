# === xAct_fe_main.py ===
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from xAct_action import SequentialAction, ParallelAction
from xAct_robot_steer import Robot

# --- Initialize hub and robot (ports match fe_car.py) ---
hub = TechnicHub()
robot = Robot(
    hub,
    drive_port=Port.A,
    steer_port=Port.B,
    steer_dir=Direction.COUNTERCLOCKWISE,
)

# Optional: run odometry in parallel if you use X/Y navigation.
odometry = robot.odometry_action()

# Available action factories on Robot:
#   - odometry_action()
#   - drive_distance_action(distance_mm, speed=60, heading=None, ...)
#   - drive_to_point_action(x_mm, y_mm, speed=60, ...)
#   - turn_to_heading_action(heading_abs, speed=30, ...)
#   - drive_dc_action(speed, duration_ms=None, ...)
#   - steer_action(target_deg, wait=False)
#   - reset_odometry_action(x_mm, y_mm, heading_abs)
#   - wait_action(duration_ms)
#   - sound_action()
#   - print_pose_action()


def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(0, 0, 0),
            robot.drive_distance_action(2000, speed=100, heading=0),
            robot.print_pose_action(),
        ])
    ]


# --- Run single mission ---
actions = mission(robot)

if actions:
    try:
        hub.speaker.beep()
        wait(300)
    except AttributeError:
        pass

while actions:
    odometry.update()
    for action in actions[:]:
        if action.update():
            actions.remove(action)
