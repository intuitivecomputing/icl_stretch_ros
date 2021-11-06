#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap
import numpy as np
import os

import hello_helpers.hello_misc as hm
import hello_helpers.hello_ros_viz as hr

import stretch_funmap.merge_maps as mm
import stretch_funmap.navigate as nv
import stretch_funmap.mapping as ma
import stretch_funmap.segment_max_height_image as sm
import stretch_funmap.navigation_planning as na
import stretch_funmap.manipulation_planning as mp

# move_to_pose :
# head_joint_names = ["joint_head_pan", "joint_head_tilt"]
# end_of_arm_joint_names = [
#     "joint_wrist_yaw",
#     "joint_gripper_finger_left",
#     "joint_gripper_finger_right",
# ]
# ripper_joint_names = [
#     "joint_gripper_finger_left",
#     "joint_gripper_finger_right",
#     "gripper_aperture",
# ]
# "wrist_extension"
# "joint_lift"
# incrementing_joint_names = ["translate_mobile_base", "rotate_mobile_base"]


class RobotInterface:
    def __init__(self, node):
        self.move_base = nv.MoveBase(node)


class RetrieveInstrumentsNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None

        self.wrist_camera_topic = "/wrist_camera/depth/color/points"

        self.workbench_goal = Pose([0.0, 0.1, 0.0], [0, 0, 0.0, 0.0])
        self.shelf_goal = Pose([1.3, 2.4, 0.0], [0, 0, 0.0, 1.57])

    def _init_params(self):
        self.debug_directory = rospy.get_param("~debug_directory")
        rospy.loginfo(
            "Using the following directory for debugging files: {0}".format(
                self.debug_directory
            )
        )

    def _init_subscribers(self):
        # Subscribe
        # wrist_camera_topic = "/wrist_camera/depth/color/points"
        # self.wrist_camera_subscriber = rospy.Subscriber(
        #     wrist_camera_topic,
        #     PointCloud2,
        #     self.wrist_camera_callback,
        # )

        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_states_callback
        )

    def _init_services(self):
        # Services
        self.trigger_grasp_object_service = rospy.Service(
            "trigger_grasp_object",
            Trigger,
            self.trigger_grasp_object_callback,
        )

        # RealSense services
        default_service = "/camera/switch_to_default_mode"
        high_accuracy_service = "/camera/switch_to_high_accuracy_mode"
        rospy.loginfo(
            "Node "
            + self.node_name
            + " waiting to connect to "
            + default_service
            + " and "
            + high_accuracy_service
        )
        rospy.wait_for_service(default_service)
        rospy.loginfo(
            "Node " + self.node_name + " connected to " + default_service
        )
        self.trigger_d435i_default_mode_service = rospy.ServiceProxy(
            "/camera/switch_to_default_mode", Trigger
        )
        rospy.wait_for_service(high_accuracy_service)
        rospy.loginfo(
            "Node " + self.node_name + " connected to" + high_accuracy_service
        )
        self.trigger_d435i_high_accuracy_mode_service = rospy.ServiceProxy(
            high_accuracy_service, Trigger
        )

        # Robot services
        # self.calibrate_service = rospy.ServiceProxy("/calibrate_the_robot", Trigger)

    # def move_to_initial_configuration(self):

    #     ma.stow_and_lower_arm(self)
    #     initial_pose = {
    #         "wrist_extension": 0.01,
    #         "joint_wrist_yaw": 0.0,
    #         "joint_lift": 0.5,
    #         "gripper_aperture": 0.0,  # 0.125,
    #     }

    #     rospy.loginfo("Move to the initial configuration for drawer opening.")
    #     self.move_to_pose(initial_pose)

    def gripper_close(self):
        self.move_to_pose(dict(gripper_aperture=0.0))

    def gripper_open(self):
        self.move_to_pose(dict(gripper_aperture=1.0))

    def drive(self, forward_m):
        tolerance_distance_m = 0.005
        if forward_m > 0:
            at_goal = self.move_base.forward(
                forward_m,
                detect_obstacles=False,
                tolerance_distance_m=tolerance_distance_m,
            )
        else:
            at_goal = self.move_base.backward(
                forward_m,
                detect_obstacles=False,
                tolerance_distance_m=tolerance_distance_m,
            )

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(
            joint_states
        )
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(
            joint_states
        )
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(
            joint_states
        )

    def trigger_grasp_object_callback(self, request):
        retrieved = False

    def get_wrist_camera_point_cloud(self):
        return rospy.wait_for_message(self.wrist_camera_subscriber, PointCloud2)

    def main(self):
        hm.HelloNode.main(
            self,
            "retrieve_instruments",
            "retrieve_instruments",
            wait_for_first_pointcloud=False,
        )

        self._init_params()
        self._init_subscribers()
        self._init_services()
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    try:
        parser = ap.ArgumentParser(
            description="Retrieve Instruments behavior for stretch."
        )
        args, unknown = parser.parse_known_args()
        node = RetrieveInstrumentsNode()()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo("interrupt received, so shutting down")
