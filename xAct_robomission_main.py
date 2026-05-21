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
    arm_left_port=None,
    arm_right_port=None
)
odometry = robot.odometry_action()


def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            robot.drive_to_point_action(x=600, y=0, speed=100),
            robot.drive_to_point_action(x=600, y=600, speed=100),
            robot.drive_to_point_action(x=0, y=600, speed=100),
            robot.drive_to_point_action(x=0, y=0, speed=100),
            robot.turn_to_heading_action(0),
            robot.drive_to_point_action(x=600 - 80, y=0, speed=100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(x=None, y=600 - 160, speed=100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(x=0 + 80, y=None, speed=100),
            robot.one_wheel_turn_action("L", 90, 1000),
            robot.drive_to_point_action(x=0, y=0, speed=100),
            robot.turn_to_heading_action(0),
        ])
    ]

actions = mission(robot)

robot.beep()
wait(500)

run_actions(robot, actions, odometry)
