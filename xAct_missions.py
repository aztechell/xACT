from xAct_action import SequentialAction, ParallelAction

def mission1(robot):
    return [
        SequentialAction(robot, [
            robot.reset_odometry_action(x=0, y=0, heading=0),
            ParallelAction(robot, [
                robot.drive_to_point_action(x=500, y=0, speed=70),
                robot.arm_action("L", speed=50, angle=50),
            ]), 
            robot.single_wheel_action("R", speed=70, angle=60),
        ])
    ]

def mission2(robot):
    return [
        SequentialAction(robot, [
            robot.straight_action(distance=820, speed=100),
            robot.single_wheel_action("L", speed=-60, angle=200),
            ParallelAction(robot, [
                robot.straight_action(distance=120, speed=70),
                robot.arm_action("R", speed=60, angle=100),
            ]), 
            robot.wait_action(500),
        ])
    ]

mission_list = [mission1, mission2]
