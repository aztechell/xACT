from pybricks.hubs import PrimeHub
from pybricks.tools import wait
from pybricks.parameters import Axis, Port
from xAct_action import ParallelAction, SequentialAction
from xAct_robot import Robot

# --- Initialize hub and robot ---
hub = PrimeHub(front_side=Axis.Y, top_side=Axis.Z)
robot = Robot(
    hub,
    left_motor=Port.B,
    right_motor=Port.F,
    arm_left_port=None,
    arm_right_port=None
)
odometry = robot.odometry_action()

def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(0, 0, 0),
            robot.drive_to_point_action(_X = 60, _Y = 0, speed = 100),
            robot.drive_to_point_action(_X = 60, _Y = 60, speed = 100),
            robot.drive_to_point_action(_X = 0, _Y = 60, speed = 100),
            robot.drive_to_point_action(_X = 0, _Y = 0, speed = 100),
            robot.turn_to_heading_action(0),
            robot.drive_to_point_action(_X = 60-8, _Y = 0, speed = 100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(_X = None, _Y = 60-16, speed = 100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(_X = 0+8, _Y = None, speed = 100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(_X = 0, _Y = 0, speed = 100),
            robot.turn_to_heading_action(0),
        ])
    ]

# --- Run single mission ---
actions = mission(robot)

# Always include odometry
# actions.append(ParallelAction(robot, [robot.odometry_action()]))

hub.speaker.beep()
wait(500)

# --- Run actions sequentially ---
while actions:
    odometry.update()
    for action in actions[:]:
        if action.update():
            actions.remove(action)
