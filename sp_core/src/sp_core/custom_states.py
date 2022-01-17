#!/usr/bin/env python3
import math

import cv2
import hello_helpers.hello_misc as hm
import numpy as np
import ros_numpy as rnp
import rospy
import smach
import smach_ros
import stretch_funmap.manipulation_planning as mp
import stretch_funmap.mapping as ma
import stretch_funmap.navigate as nv
import stretch_funmap.navigation_planning as na
from cv_bridge import CvBridge, CvBridgeError
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Trigger, TriggerRequest


# def get_robot_pose(self):
#     stamped_transform = self.tf_buffer.lookup_transform(
#         "/map", "/base_link", rospy.Time(0), rospy.Duration(0.1)
#     )
#     trans_mat = rnp.numpify(stamped_transform)
#     translation = trans_mat[:3, 3]
#     rotation = tr.euler_from_matrix(trans_mat)
#     return translation, rotation
def prepare_gripper(node):
    # pose = {"wrist_extension": 0.1}
    # node.move_to_pose(pose)

    # gripper backwards stow
    pose = {"joint_wrist_yaw": 3.3}

    # gripper forward stow needs a better forward range of motion to work well
    node.move_to_pose(pose)


class WaitForOrderState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=["start", "idle"])
        self.node = node

    def execute(self, ud):
        if len(self.node.marker_list) != 0:
            rospy.loginfo(
                f"[{self.__class__.__name__}] Order received: {self.node.marker_list}"
            )
            if self.node.is_start:
                rospy.loginfo(
                    f"[{self.__class__.__name__}] Start signal received."
                )
                self.node.is_start = False
                return "start"
            else:
                rospy.loginfo(
                    f"[{self.__class__.__name__}] Waiting for start signal."
                )
        else:
            rospy.loginfo(
                f"[{self.__class__.__name__}] Waiting for robot order."
            )
        rospy.sleep(1.5)
        return "idle"


class GetPoseState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            output_keys=["robot_pose"],
        )
        self.node = node

    def execute(self, userdata):
        (x, y, r), ts = self.node.get_robot_floor_pose_xya(floor_frame="map")
        rospy.loginfo(
            f"[{self.__class__.__name__}] Get robot pose: x: {x}, y: {y}, r: {r}"
        )
        userdata.robot_pose = (x, y, r)
        return "succeeded"


class AlignState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted"],
            # input_keys=["robot_pose"],
        )
        self.node = node

    def execute(self, userdata):
        (x, y, a), ts = self.node.get_robot_floor_pose_xya(floor_frame="map")
        rospy.loginfo(f"Get robot pose: x: {x}, y: {y}, a: {a}")
        align_arm_ang = a + (np.pi / 2.0)
        turn_ang = hm.angle_diff_rad(np.pi / 2.0, align_arm_ang)
        at_goal = self.node.move_base.turn(
            turn_ang, publish_visualizations=True
        )
        if at_goal:
            rospy.loginfo(f"[{self.__class__.__name__}] Robot aligned.")
            return "succeeded"
        else:
            return "aborted"


class PreSearchState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted"],
        )
        self.node = node

    def execute(self, userdata):
        # max_wrist_extension_m = 0.5
        # gripper backwards stow
        pose = {"joint_wrist_yaw": 3.3}
        self.node.move_to_pose(pose)
        pose = {"joint_lift": 0.9}
        self.node.move_to_pose(pose)
        pose = {"wrist_extension": 0.1}
        self.node.move_to_pose(pose)
        rospy.sleep(0.5)
        rospy.loginfo(f"[{self.__class__.__name__}]")
        return "succeeded"


class PreMoveState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
        )
        self.node = node

    def execute(self, userdata):
        self.node.switch_to_position_mode_service(TriggerRequest())
        pose = {"joint_gripper_finger_left": -0.15}
        self.node.move_to_pose(pose)
        pose = {"wrist_extension": 0.01}
        self.node.move_to_pose(pose)

        # gripper backwards stow
        pose = {"joint_wrist_yaw": 3.3}

        # gripper forward stow needs a better forward range of motion to work well
        self.node.move_to_pose(pose)

        # avoid blocking the laser range finder with the gripper
        pose = {"joint_lift": 0.22}
        self.node.move_to_pose(pose)
        rospy.loginfo(f"[{self.__class__.__name__}] Stow robot")
        return "succeeded"


class SearchMarkersState(smach.State):
    def __init__(self, marker_list):
        smach.State.__init__(
            self,
            outcomes=["undetected", "detected"],
            output_keys=["detected_marker_id"],
        )
        self.marker_list = marker_list

    def execute(self, userdata):
        aruco_array = rospy.wait_for_message(
            "/fiducial_transforms", FiducialTransformArray
        )
        detections = [
            ft
            for ft in aruco_array.transforms
            if ft.fiducial_id in self.marker_list
        ]
        num_detection = len(detections)
        if num_detection == 0:
            rospy.loginfo(f"[{self.__class__.__name__}] No marker detected.")
            return "undetected"
        rospy.loginfo(
            f"[{self.__class__.__name__}] Marker #{detections[0].fiducial_id} detected."
        )
        # userdata.detected_markers = detections
        userdata.detected_marker_id = detections[0].fiducial_id
        return "detected"


class PreDetectState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self, outcomes=["succeeded"], input_keys=["detected_marker_id"]
        )
        self.node = node

    def execute(self, userdata):
        marker_id = userdata.detected_marker_id

        grasp_center_frame_id = "link_grasp_center"
        cam_frame_id = "wrist_camera_color_optical_frame"
        target_frame_id = f"fiducial_{marker_id}"
        base_frame_id = "base_link"

        # target_frame in cam_frame
        translation = self.node.lookup_transform_mat(
            cam_frame_id,
            target_frame_id,
        )[:3, 3]

        rospy.loginfo(
            f"[{self.__class__.__name__}] fiducial_{marker_id} at {translation} in {cam_frame_id}"
        )
        camera_tolerance = (0.05, 0.05, 0.5)
        if np.abs(translation[2] - camera_tolerance[2]) >= 0.02:
            # print(self.node.wrist_position)
            pose = {
                "wrist_extension": self.node.wrist_position
                + (translation[2] - camera_tolerance[2])
            }
            self.node.move_to_pose(pose)
        if np.abs(translation[0]) >= camera_tolerance[0]:
            self.node.move_base.forward(-translation[0], detect_obstacles=False)
        if np.abs(translation[1] + camera_tolerance[1]) >= 0.02:
            pose = {
                "joint_lift": self.node.lift_position - (translation[1] + 0.05)
            }
            self.node.move_to_pose(pose)

        return "succeeded"


class DetectState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted"],
            input_keys=["detected_marker_id"],
            output_keys=["target", "target_frame"],
        )
        self.node = node
        self.min_cloud_points = 15

    def execute(self, userdata):
        marker_id = userdata.detected_marker_id
        target_frame_id = f"fiducial_{marker_id}"

        # set wrist cam to high accuracy
        rospy.set_param("/wrist_camera/stereo_module/visual_preset", "3")

        # move gripper to pregrasp position
        # pose = {"joint_wrist_yaw": 0.0}
        # self.node.move_to_pose(pose)
        pose = {"gripper_aperture": 0.125}
        self.node.move_to_pose(pose)
        rospy.loginfo(
            f"[{self.__class__.__name__}]  wait 2sec for sensor to stabalize"
        )
        rospy.sleep(2.0)

        cloud_msg = rospy.wait_for_message(
            "/wrist_camera/depth/color/points", PointCloud2
        )
        cloud_frame = cloud_msg.header.frame_id
        point_cloud = rnp.numpify(cloud_msg)
        xyz = rnp.point_cloud2.get_xyz_points(point_cloud)  # (N,3)
        # target_frame in cloud_frame
        translation = self.node.lookup_transform_mat(
            cloud_frame,
            target_frame_id,
        )[:3, 3]

        marker_coord = translation + [0.03, 0.03, -0.03]
        detect_box = (0.12, 0.12)

        cloud_mask = []
        dxyz = xyz - marker_coord.reshape(1, 3)
        for p in dxyz:
            if (
                p[2] < 0
                and (p[0] > 0 and p[0] < detect_box[0])
                and (p[1] > 0 and p[1] < detect_box[1])
            ):
                cloud_mask.append(True)
            else:
                cloud_mask.append(False)
        filtered_cloud = xyz[cloud_mask]
        if len(filtered_cloud) <= self.min_cloud_points:
            rospy.loginfo(f"[{self.__class__.__name__}] too few points")
            return "aborted"
        center_of_mass = np.average(filtered_cloud, axis=0)
        rospy.loginfo(
            f"[{self.__class__.__name__}] detected COM at {center_of_mass} in {cloud_frame}"
        )
        userdata.target = center_of_mass
        userdata.target_frame = cloud_frame

        self.node.update_vis_markers(cloud_frame, center_of_mass)

        # set wrist cam to default
        rospy.set_param("/wrist_camera/stereo_module/visual_preset", "1")
        return "succeeded"


class GraspState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["target", "target_frame"],
        )
        self.node = node

    def execute(self, userdata):
        target = userdata.target
        target_frame = userdata.target_frame
        grasp_center_frame = "link_grasp_center"
        base_frame = "base_link"
        wrist_extension_offset_m = 0.03
        forward_offset_m = 0.0  # 0.02

        target_to_base_mat = self.node.lookup_transform_mat(
            base_frame,
            target_frame,
        )

        grasp_to_base_mat = self.node.lookup_transform_mat(
            base_frame,
            grasp_center_frame,
        )

        grasp_target = np.array([0.0, 0.0, 0.0, 1.0])
        grasp_target[:3] = target
        # target in base frame
        grasp_target_in_base_frame = np.matmul(
            target_to_base_mat, grasp_target
        )[:3]
        grasp_center_in_base_frame = grasp_to_base_mat[:3, 3]
        translation = grasp_target_in_base_frame - grasp_center_in_base_frame
        # print(grasp_target_in_base_frame - grasp_center_in_base_frame)
        # move in x / forward
        self.node.move_base.forward(
            translation[0] + forward_offset_m, detect_obstacles=False
        )
        # move z / lift
        pose = {"joint_lift": self.node.lift_position + translation[2]}
        self.node.move_to_pose(pose)
        # move y / -side
        pose = {
            "wrist_extension": self.node.wrist_position
            - translation[1]
            + wrist_extension_offset_m
        }
        self.node.move_to_pose(pose)

        rospy.loginfo(
            f"[{self.__class__.__name__}] grasping at {translation} in {grasp_center_frame}"
        )
        rospy.sleep(1.0)
        pose = {"gripper_aperture": -0.1}
        self.node.move_to_pose(pose)

        # self.node.update_vis_markers(base_frame, grasp_target_in_base_frame)
        # self.node.update_vis_markers(base_frame, grasp_center_in_base_frame)
        return "succeeded"


class MagnetState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["target", "target_frame"],
        )
        self.node = node

    def execute(self, userdata):
        pose = {"joint_wrist_yaw": 1.57}
        self.node.move_to_pose(pose)

        target = userdata.target
        target_frame = userdata.target_frame
        grasp_center_frame = "link_magnet"
        base_frame = "base_link"
        wrist_extension_offset_m = 0.05
        forward_offset_m = 0.06

        target_to_base_mat = self.node.lookup_transform_mat(
            base_frame,
            target_frame,
        )

        grasp_to_base_mat = self.node.lookup_transform_mat(
            base_frame,
            grasp_center_frame,
        )

        grasp_target = np.array([0.0, 0.0, 0.0, 1.0])
        grasp_target[:3] = target
        # target in base frame
        grasp_target_in_base_frame = np.matmul(
            target_to_base_mat, grasp_target
        )[:3]
        grasp_center_in_base_frame = grasp_to_base_mat[:3, 3]
        translation = grasp_target_in_base_frame - grasp_center_in_base_frame
        # print(grasp_target_in_base_frame - grasp_center_in_base_frame)
        # move in x / forward
        self.node.move_base.forward(
            translation[0] + forward_offset_m, detect_obstacles=False
        )
        # move z / lift
        pose = {"joint_lift": self.node.lift_position + translation[2]}
        self.node.move_to_pose(pose)

        # move y / -side
        pose = {
            "wrist_extension": self.node.wrist_position
            - translation[1]
            + wrist_extension_offset_m
        }
        self.node.move_to_pose(pose)

        rospy.loginfo(
            f"[{self.__class__.__name__}] grasping at {translation} in {grasp_center_frame}"
        )

        self.node.trigger_magnet_service(TriggerRequest())
        rospy.sleep(1.0)
        # pose = {"gripper_aperture": -0.1}
        # self.node.move_to_pose(pose)

        # self.node.update_vis_markers(base_frame, grasp_target_in_base_frame)
        # self.node.update_vis_markers(base_frame, grasp_center_in_base_frame)
        return "succeeded"


class PostGraspState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
        )
        self.node = node

    def execute(self, userdata):
        rospy.sleep(1.0)
        post_grasp_lift_m = 0.01
        print(self.node.lift_position)
        pose = {"joint_lift": self.node.lift_position + post_grasp_lift_m}
        self.node.move_to_pose(pose)
        rospy.sleep(0.5)
        pose = {"wrist_extension": 0.1}
        self.node.move_to_pose(pose)
        pose = {"joint_wrist_yaw": 3.6}
        self.node.move_to_pose(pose)

        pose = {"joint_lift": 0.25}
        self.node.move_to_pose(pose)
        pose = {"gripper_aperture": 0.125}
        self.node.move_to_pose(pose)
        return "succeeded"


class PostMagnetState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
        )
        self.node = node

    def execute(self, userdata):
        rospy.sleep(1.0)
        post_grasp_lift_m = 0.01
        print(self.node.lift_position)
        pose = {"joint_lift": self.node.lift_position + post_grasp_lift_m}
        self.node.move_to_pose(pose)
        rospy.sleep(0.5)
        pose = {"wrist_extension": 0.1}
        self.node.move_to_pose(pose)
        pose = {"joint_wrist_yaw": -1.57}
        self.node.move_to_pose(pose)

        pose = {"joint_lift": 0.25}
        self.node.move_to_pose(pose)

        self.node.trigger_magnet_service(TriggerRequest())
        rospy.sleep(1.0)
        # pose = {"gripper_aperture": 0.125}
        # self.node.move_to_pose(pose)
        return "succeeded"
