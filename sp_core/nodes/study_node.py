#!/usr/bin/env python3

import argparse as ap
import math
import multiprocessing
import os
import sys
import threading
import time

import actionlib
import hello_helpers.hello_misc as hm
import numpy as np
import rospy
import stretch_funmap.manipulation_planning as mp
import stretch_funmap.navigate as nv
import tf2_ros
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

# from fiducial_msgs import FiducialTransformArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from std_srvs.srv import (
    SetBool,
    SetBoolRequest,
    Trigger,
    TriggerRequest,
    TriggerResponse,
)
from trajectory_msgs.msg import JointTrajectoryPoint


class StudyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        self.task = False
        self.task_thread = None
        self.fiducials = []

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

    def fiducial_callback(self, data):
        # fiducials = data.transforms÷
        # if len(fiducials) >0:
        #     min(fiducials, key=)
        pass

    def lower_tool_until_contact(self):
        rospy.loginfo("lower_tool_until_contact")
        trigger_request = TriggerRequest()
        trigger_result = self.trigger_lower_until_contact_service(
            trigger_request
        )
        rospy.loginfo("trigger_result = {0}".format(trigger_result))

    def wipe_in(self):
        rospy.loginfo("wipe_in")
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position - 0.2
            pose = {"wrist_extension": wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            rospy.logerr("wipe_in: self.wrist_position is None!")
            return False

    def wipe_out(self):
        rospy.loginfo("wipe_out")
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position + 0.22
            pose = {"wrist_extension": wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            rospy.logerr("wipe_out: self.wrist_position is None!")
            return False

    def pre_wipe(self):
        rospy.loginfo("Prewipe.")
        pose = {"joint_lift": 0.8}
        self.move_to_pose(pose)
        pose = {"wrist_extension": 0.2}
        self.move_to_pose(pose)

    def stow(self):
        rospy.loginfo("Stow.")
        pose = {"joint_head_tilt": 0.4}
        self.move_to_pose(pose)
        pose = {"joint_lift": 0.8}
        self.move_to_pose(pose)
        pose = {"wrist_extension": 0.01}
        self.move_to_pose(pose)
        self.move_to_pose({"joint_wrist_yaw": 0.0})
        self.move_to_pose({"gripper_aperture": 0.0})

    def rest(self):
        rospy.loginfo("Rest.")
        pose = {"wrist_extension": 0.01}
        self.move_to_pose(pose)
        pose = {"joint_lift": 0.4}
        self.move_to_pose(pose)
        self.move_to_pose({"joint_wrist_yaw": 3})
        self.move_to_pose({"gripper_aperture": 0.05})

    def clean(self, position):
        lift_to_surface_m = 0.75
        self.stow()
        if position == "mid":
            self.pre_wipe()
            # self.lower_tool_until_contact()
            pose = {"joint_lift": lift_to_surface_m}
            self.move_to_pose(pose)
            if not self.task:
                return
            self.wipe_out()
            if not self.task:
                return
            self.wipe_in()
            if not self.task:
                return
            self.move_to_pose({"joint_lift": 0.8})
            if not self.task:
                return
            self.move_to_pose({"wrist_extension": 0.01})
            if not self.task:
                return
        elif position == "left":
            self.pre_wipe()
            # self.lower_tool_until_contact()
            pose = {
                "joint_lift": lift_to_surface_m,
                "joint_wrist_yaw": np.deg2rad(15),
            }
            self.move_to_pose(pose)
            self.wipe_out()
            if not self.task:
                return
            self.wipe_in()
            if not self.task:
                return
            self.move_to_pose({"joint_lift": 0.8})
            if not self.task:
                return
            self.move_to_pose({"wrist_extension": 0.01})
            if not self.task:
                return
        elif position == "right":
            self.pre_wipe()
            # self.lower_tool_until_contact()
            pose = {
                "joint_lift": lift_to_surface_m,
                "joint_wrist_yaw": np.deg2rad(-15),
            }
            self.move_to_pose(pose)
            self.wipe_out()
            if not self.task:
                return
            self.wipe_in()
            if not self.task:
                return
            self.move_to_pose({"joint_lift": 0.8})
            if not self.task:
                return
            self.move_to_pose({"wrist_extension": 0.01})
            if not self.task:
                return
        elif position == "left_side":
            self.move_to_pose({"joint_wrist_yaw": np.deg2rad(60)})
            if not self.task:
                return
            self.move_to_pose({"wrist_extension": 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m + 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m - 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m + 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m - 0.2})

        elif position == "right_side":
            self.move_to_pose({"joint_wrist_yaw": np.deg2rad(-60)})
            if not self.task:
                return
            self.move_to_pose({"wrist_extension": 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m + 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m - 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m + 0.2})
            if not self.task:
                return
            self.move_to_pose({"joint_lift": lift_to_surface_m - 0.2})

    def random_clean(self):
        pos = np.random.choice(
            ["mid", "left", "right"]
            # ["mid", "left", "right", "left_side", "right_side"]
        )
        # pos = np.random.choice(["mid"])

        self.clean(position=pos)

        self.stow()

    def trigger_stow_callback(self, request):
        self.stow()
        return TriggerResponse(success=True, message="Completed stow!")

    def trigger_rest_callback(self, request):
        self.rest()
        return TriggerResponse(success=True, message="Completed rest!")

    def trigger_clean_surface_callback(self, request):
        self.task = not self.task
        # if not self.task and self.task_thread is not None:
        #     self.task_thread.terminate()
        msg = SetBoolRequest()
        msg.data = not self.task
        self.trigger_stop_service(msg)
        msg = SetBoolRequest()
        msg.data = self.task
        resp = self.trigger_robot_gaze_service(msg)
        rospy.loginfo(f"Trigger robot gaze: {resp}")

        rospy.loginfo(f"Task state: {self.task}")

        return TriggerResponse(
            success=True, message="Completed surface cleaning!"
        )

    # def look_at_surface(self):
    #     self.manipulation_view = mp.ManipulationView(
    #         self.tf2_buffer, self.debug_directory
    #     )
    #     manip = self.manipulation_view
    #     manip.move_head(self.move_to_pose)
    #     manip.update(self.point_cloud, self.tf2_buffer)
    #     if self.debug_directory is not None:
    #         dirname = self.debug_directory + "clean_surface/"
    #         # If the directory does not already exist, create it.
    #         if not os.path.exists(dirname):
    #             os.makedirs(dirname)
    #         filename = "look_at_surface_" + hm.create_time_string()
    #         manip.save_scan(dirname + filename)
    #     else:
    #         rospy.loginfo(
    #             "CleanSurfaceNode: No debug directory provided, so debugging data will not be saved."
    #         )

    def main(self):
        hm.HelloNode.main(
            self,
            "clean_surface",
            "clean_surface",
            wait_for_first_pointcloud=False,
        )

        self.debug_directory = rospy.get_param(
            "~debug_directory", "/home/hello-robot/stretch_user/debug"
        )
        rospy.loginfo(
            "Using the following directory for debugging files: {0}".format(
                self.debug_directory
            )
        )

        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_states_callback
        )

        # self.aruco_subscriber = rospy.Subscriber(
        #     "/fiducial_transforms",
        #     FiducialTransformArray,
        #     self.fiducial_callback,
        # )

        self.trigger_clean_surface_service = rospy.Service(
            "/clean_surface/trigger_clean_surface",
            Trigger,
            self.trigger_clean_surface_callback,
        )
        self.trigger_stow_service = rospy.Service(
            "/robot/stow", Trigger, self.trigger_stow_callback
        )
        self.trigger_rest_service = rospy.Service(
            "/robot/rest", Trigger, self.trigger_rest_callback
        )

        rospy.wait_for_service("/robot_face/gaze")
        rospy.loginfo(
            "Node " + self.node_name + " connected to /robot_face/gaze."
        )
        self.trigger_robot_gaze_service = rospy.ServiceProxy(
            "/robot_face/gaze", SetBool
        )

        rospy.wait_for_service("/runstop")
        rospy.loginfo("Node " + self.node_name + " connected to /runstop.")
        self.trigger_stop_service = rospy.ServiceProxy("/runstop", SetBool)

        rospy.wait_for_service("/funmap/trigger_reach_until_contact")
        rospy.loginfo(
            "Node "
            + self.node_name
            + " connected to /funmap/trigger_reach_until_contact."
        )
        self.trigger_reach_until_contact_service = rospy.ServiceProxy(
            "/funmap/trigger_reach_until_contact", Trigger
        )

        rospy.wait_for_service("/funmap/trigger_lower_until_contact")
        rospy.loginfo(
            "Node "
            + self.node_name
            + " connected to /funmap/trigger_lower_until_contact."
        )
        self.trigger_lower_until_contact_service = rospy.ServiceProxy(
            "/funmap/trigger_lower_until_contact", Trigger
        )

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
            default_service, Trigger
        )
        rospy.wait_for_service(high_accuracy_service)
        rospy.loginfo(
            "Node " + self.node_name + " connected to" + high_accuracy_service
        )
        self.trigger_d435i_high_accuracy_mode_service = rospy.ServiceProxy(
            high_accuracy_service, Trigger
        )

        self.stow()

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if (
                (self.lift_position is not None)
                and (self.wrist_position is not None)
                and self.task
            ):
                self.random_clean()
                # print(self.task_thread)
                # if self.task_thread is None:
                #     self.task_thread = multiprocessing.Process(
                #         target=self.random_clean, daemon=True
                #     )
                #     self.task_thread.start()
                #     self.task_thread.join()
                # elif self.task_thread.is_alive():
                #     if not self.task:
                #         self.task_thread.terminate()
                #         self.task_thread = None

            rate.sleep()


if __name__ == "__main__":
    try:
        parser = ap.ArgumentParser(
            description="Clean Surface behavior for stretch."
        )
        args, unknown = parser.parse_known_args()
        node = StudyNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo("interrupt received, so shutting down")
