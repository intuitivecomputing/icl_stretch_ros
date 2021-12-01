#!/usr/bin/env python3

import time
import math
import stretch_body.robot
from stretch_body.hello_utils import ThreadServiceExit

vel_fast_m = 0.3
accel_fast_m = 0.4

robot = stretch_body.robot.Robot()
robot.startup()

if not robot.is_calibrated():
    robot.home()  # blocking


def move(dist):
    robot.base.translate_by(x_m=dist, v_m=vel_fast_m, a_m=accel_fast_m)
    robot.push_command()
    time.sleep(dist / vel_fast_m + 2)  # wait


def rot(rad):
    robot.base.rotate_by(x_r=rad)
    robot.push_command()
    time.sleep(2.0)  # wait


try:

    # Get a snapshot of the robot status data
    status = robot.get_status()
    robot.pretty_print()

    robot.end_of_arm.stow()
    robot.lift.move_to(x_m=0.8)

    # Make sure wrist yaw is done before exiting
    # while (
    #     robot.end_of_arm.motors["wrist_yaw"].motor.is_moving()
    #     or not robot.lift.motor.status["near_pos_setpoint"]
    # ):
    #     time.sleep(0.1)
    time.sleep(2.0)

    move(2.5)
    rot(-math.pi / 2)
    move(5)


except (KeyboardInterrupt, SystemExit, ThreadServiceExit):
    pass

robot.stop()
