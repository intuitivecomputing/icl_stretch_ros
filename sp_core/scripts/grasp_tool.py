#!/usr/bin/env python

from __future__ import print_function

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import threading
import argparse as ap

from tf.transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_from_matrix,
)
import tf2_ros
import math
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp

import numpy as np
import ros_numpy as rn


class GraspToolNode(hm.HelloNode):
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
        self.br = tf2_ros.TransformBroadcaster()

        marker = Marker()
        self.target_point = None

        self.pose = None
        # self.transform_listener = rospy.tf2.tranformlistener

        num_pan_angles = 5

        # looking out along the arm
        middle_pan_angle = -math.pi / 2.0

        look_around_range = math.pi / 3.0
        min_pan_angle = middle_pan_angle - (look_around_range / 2.0)
        max_pan_angle = middle_pan_angle + (look_around_range / 2.0)
        pan_angle = min_pan_angle
        pan_increment = look_around_range / float(num_pan_angles - 1.0)
        self.pan_angles = [
            min_pan_angle + (i * pan_increment) for i in range(num_pan_angles)
        ]
        self.pan_angles = self.pan_angles + self.pan_angles[1:-1][::-1]
        self.prev_pan_index = 0

        self.move_lock = threading.Lock()

        with self.move_lock:
            self.handover_goal_ready = False
            # home camera
            # pose = {"joint_head_pan": middle_pan_angle}
            # self.move_to_pose(pose)

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

        """The following keys can be in the dictionary
        joint_lift
        wrist_extension
        joint_wrist_yaw
        gripper_aperture
        translate_mobile_base
        joint_head_pan

        """

    def look_around_callback(self):

        # Cycle the head back and forth looking for a person to whom
        # to handout the object.
        with self.move_lock:
            pan_index = (self.prev_pan_index + 1) % len(self.pan_angles)
            pan_angle = self.pan_angles[pan_index]
            pose = {"joint_head_pan": pan_angle}
            self.move_to_pose(pose)
            self.prev_pan_index = pan_index

    def move_to_initial_configuration(self):
        initial_pose = {
            "wrist_extension": 0.01,
            "joint_wrist_yaw": 0.0,
            "joint_lift": 0.5,
            "gripper_aperture": 0.0,  # 0.125,
        }

        rospy.loginfo("Move to the initial configuration.")
        self.move_to_pose(initial_pose)

    def gripper_close(self):
        self.move_to_pose(dict(gripper_aperture=0.0))

    def gripper_open(self):
        self.move_to_pose(dict(gripper_aperture=1.0))

    def trigger_grasp_object_callback(self, request):
        with self.move_lock:
            if self.handover_goal_ready:
                # First, retract the wrist in preparation for handing out an object.
                # pose = {"wrist_extension": 0.01, "gripper_aperture": 0.125}
                # self.move_to_pose(pose)
                self.move_to_initial_configuration()

                if self.handover_goal_ready:
                    pose = {"joint_lift": self.lift_goal_m}
                    self.move_to_pose(pose)
                    tolerance_distance_m = 0.01
                    at_goal = self.move_base.forward(
                        self.mobile_base_forward_m,
                        detect_obstacles=False,
                        tolerance_distance_m=tolerance_distance_m,
                    )
                    pose = {"wrist_extension": self.wrist_goal_m}
                    self.move_to_pose(pose)
                    self.handover_goal_ready = False

                    # # close gripper and lift
                    self.gripper_close()
                    rospy.sleep(2)
                    self.move_to_pose(dict(joint_lift=self.lift_goal_m + 0.02))
                    rospy.sleep(2)
                    self.move_to_pose(dict(wrist_extension=0.1))
                    self.move_to_pose(
                        dict(
                            joint_lift=0.25,
                            joint_wrist_yaw=math.pi * 1.1,
                        )
                    )
                    self.gripper_open()
                    self.move_to_pose(dict(joint_lift=0.5))
                    rospy.sleep(5)
                    self.move_to_initial_configuration()

                return TriggerResponse(
                    success=True, message="Completed object grasp!"
                )

    def trigger_magnet_grasp_object_callback(self, request):
        print("start grasping")
        with self.move_lock:
            if self.handover_goal_ready:
                initial_pose = {
                    "wrist_extension": 0.01,
                    "joint_wrist_yaw": 0.0,
                    "joint_lift": 0.5,
                    "gripper_aperture": 0.0,
                }

                rospy.loginfo("Move to the initial configuration.")
                self.move_to_pose(initial_pose)

                if self.handover_goal_ready:
                    lift_goal_m = self.lift_goal_m
                    wrist_goal_m = self.wrist_goal_m
                    pose = {"joint_lift": lift_goal_m}
                    self.move_to_pose(pose)
                    tolerance_distance_m = 0.01
                    at_goal = self.move_base.forward(
                        self.mobile_base_forward_m,
                        detect_obstacles=False,
                        tolerance_distance_m=tolerance_distance_m,
                    )
                    pose = {"wrist_extension": wrist_goal_m}
                    self.move_to_pose(pose)
                    self.handover_goal_ready = False

                    # # close gripper and lift
                    # self.gripper_close()
                    rospy.sleep(2)
                    self.move_to_pose(dict(joint_lift=lift_goal_m + 0.02))
                    rospy.sleep(2)
                    self.move_to_pose(dict(wrist_extension=0.1))
                    self.move_to_pose(
                        dict(
                            joint_lift=0.25,
                            joint_wrist_yaw=math.pi * 1.1,
                        )
                    )
                    # self.gripper_open()
                    self.move_to_pose(dict(joint_lift=0.5))
                    rospy.sleep(5)
                    self.move_to_pose(dict(joint_wrist_yaw=0.0))
                    # self.move_to_initial_configuration()

                return TriggerResponse(
                    success=True, message="Completed object grasp!"
                )

    def publish_tool_tf(self, msg, frame_id):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/camera_color_optical_frame"
        t.child_frame_id = frame_id
        t.transform.translation = msg.position
        t.transform.rotation = msg.orientation

        self.br.sendTransform(t)

    def marker_pose_callback(self, marker_array):
        with self.move_lock:
            if len(marker_array.markers) == 0:
                rospy.loginfo("No marker detected!")
            else:
                for marker in marker_array.markers:
                    if marker.text == "target_tool":
                        target_position = marker.pose.position
                        self.target_point = PointStamped()
                        self.target_point.point = target_position
                        header = self.target_point.header
                        header.stamp = marker.header.stamp
                        header.frame_id = marker.header.frame_id
                        header.seq = marker.header.seq
                        print("******* new mouth point received *******")

                        lookup_time = rospy.Time(
                            0
                        )  # return most recent transform
                        timeout_ros = rospy.Duration(0.1)

                        # self.tf2_listener = tf2_ros.TransformListener(
                        #     self.tf2_buffer
                        # )

                        grasp_center_frame_id = "link_grasp_center"
                        target_frame_id = "target_tool"
                        # camera_frame_id = "camera_color_optical_frame"
                        cam_frame_id = self.target_point.header.frame_id[1:]
                        base_frame_id = "base_link"

                        stamped_transform = self.tf2_buffer.lookup_transform(
                            base_frame_id,
                            cam_frame_id,
                            lookup_time,
                            timeout_ros,
                        )
                        points_in_cam_frame_to_base_frame_mat = rn.numpify(
                            stamped_transform.transform
                        )
                        camera_to_base_mat = (
                            points_in_cam_frame_to_base_frame_mat
                        )

                        stamped_transform = self.tf2_buffer.lookup_transform(
                            base_frame_id,
                            grasp_center_frame_id,
                            lookup_time,
                            timeout_ros,
                        )
                        grasp_center_to_base_mat = rn.numpify(
                            stamped_transform.transform
                        )

                        target_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                        target_camera_xyz[:3] = rn.numpify(
                            self.target_point.point
                        )[:3]

                        target_xyz = np.matmul(
                            camera_to_base_mat, target_camera_xyz
                        )[:3]

                        # target_xyz = target_to_base_mat[:, 3][:3]
                        fingers_xyz = grasp_center_to_base_mat[:, 3][:3]

                        target_offset_xyz = np.array([0.0, 0.1, -0.16])

                        target_xyz += target_offset_xyz
                        fingers_error = target_xyz - fingers_xyz
                        print("fingers_error =", fingers_error)

                        delta_forward_m = fingers_error[0]
                        delta_extension_m = -fingers_error[1]
                        delta_lift_m = fingers_error[2]

                        max_lift_m = 1.0
                        lift_goal_m = self.lift_position + delta_lift_m
                        lift_goal_m = min(max_lift_m, lift_goal_m)
                        self.lift_goal_m = lift_goal_m

                        self.mobile_base_forward_m = delta_forward_m

                        max_wrist_extension_m = 0.5
                        wrist_goal_m = self.wrist_position + delta_extension_m

                        # if handoff_object:
                        # attempt to handoff the object by keeping distance
                        # between the object and the mouth distance
                        # wrist_goal_m = wrist_goal_m - 0.3 # 30cm from the mouth
                        # wrist_goal_m = (
                        #     wrist_goal_m   - 0.9  # 25cm from the mouth
                        # )
                        wrist_goal_m = max(0.0, wrist_goal_m)

                        self.wrist_goal_m = min(
                            max_wrist_extension_m, wrist_goal_m
                        )

                        self.handover_goal_ready = True

    def main(self):
        hm.HelloNode.main(
            self,
            "grasp_object",
            "grasp_object",
            wait_for_first_pointcloud=False,
        )

        self.debug_directory = rospy.get_param("~debug_directory")
        rospy.loginfo(
            "Using the following directory for debugging files: {0}".format(
                self.debug_directory
            )
        )

        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_states_callback
        )

        self.trigger_grasp_object_service = rospy.Service(
            "/grasp_object/trigger_grasp_object",
            Trigger,
            self.trigger_grasp_object_callback,
        )
        self.trigger_magnet_grasp_object_service = rospy.Service(
            "/grasp_object/trigger_magnet_grasp_object",
            Trigger,
            self.trigger_magnet_grasp_object_callback,
        )
        self.marker_pose_subscriber = rospy.Subscriber(
            "/aruco/marker_array", MarkerArray, self.marker_pose_callback
        )

        rate = rospy.Rate(self.rate)

        self.move_to_initial_configuration()
        look_around = False
        while not rospy.is_shutdown():
            if look_around:
                self.look_around_callback()
            # else:
            #     # home camera
            #     pose = {
            #         "joint_head_pan": -math.pi / 2.0,
            #         "joint_head_tilt": -math.pi / 6.0,
            #     }
            #     self.move_to_pose(pose)
            rate.sleep()


if __name__ == "__main__":
    try:

        parser = ap.ArgumentParser(
            description="Grasp Object behavior for stretch."
        )
        args, unknown = parser.parse_known_args()
        node = GraspToolNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo("interrupt received, so shutting down")
