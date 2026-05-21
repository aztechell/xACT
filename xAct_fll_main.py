from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Port
from pybricks.tools import hub_menu, wait
from xAct_action import run_actions
from xAct_robot import Robot
from xAct_missions import mission_list  # list of mission functions

hub = PrimeHub(front_side=Axis.Y, top_side=Axis.Z)
robot = Robot(
    hub,
    left_motor=Port.E,
    right_motor=Port.A,
    arm_left_port=Port.F,
    arm_right_port=Port.B
)
odometry = robot.odometry_action()

menu_items = tuple(str(i + 1) for i in range(len(mission_list)))
if not menu_items:
    raise RuntimeError("mission_list is empty. Add at least one mission in xAct_missions.py.")


def mission(robot):
    choice = hub_menu(*menu_items)
    print("Selected mission:", choice)

    index = int(choice) - 1
    mission_func = mission_list[index]
    return mission_func(robot)


while True:
    actions = mission(robot)

    robot.beep()
    wait(500)

    run_actions(robot, actions, odometry)
