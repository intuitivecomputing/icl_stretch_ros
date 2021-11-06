#!/usr/bin/env python3

import math
import numpy as np
import ros_numpy as rn

import rospy
import tf2_ros as tf
import tf.transformations as tr

from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import smach
import smach_ros
from smach import StateMachine, Sequence
from smach_ros import ServiceState, ActionServerWrapper


import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


def main():
    rospy.init_node("smach_node")
    sq_start = Sequence(
        outcomes=["succeeded", "aborted", "preempted"],
        connector_outcome="succeeded",
    )

    with sq_start:
        Sequence.add("CALIBRATE", ServiceState("/calibrate_the_robot", Trigger))
        Sequence.add(
            "MODE_POSITION", ServiceState("/switch_to_position_mode", Trigger)
        )
        Sequence.add(
            "NAV_GLOBAL_LOCALIZATION",
            ServiceState("/funmap/trigger_global_localization", Trigger),
        )

    # Execute SMACH plan
    outcome = sq_start.execute()


if __name__ == "__main__":
    main()
