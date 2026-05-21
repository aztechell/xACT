from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from xAct_action import SequentialAction, run_actions
from xAct_robot_steer import Robot

hub = TechnicHub()
robot = Robot(
    hub,
    drive_port=Port.A,
    steer_port=Port.B,
    steer_dir=Direction.COUNTERCLOCKWISE,
)

odometry = robot.odometry_action()


def mission(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            robot.drive_distance_action(2000, speed=100, heading=0),
            robot.print_pose_action(),
        ])
    ]


actions = mission(robot)

robot.beep()
wait(500)

run_actions(robot, actions, odometry)
