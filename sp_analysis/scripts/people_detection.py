#!/usr/bin/env python3
import math

import numpy as np
import ros_numpy as rn
import rospy
from easy_markers.generator import Marker, MarkerGenerator
from geometry_msgs.msg import Point, PointStamped, Vector3
from kalman_filter import Kalman
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import PointCloud2


def distance(leg1, leg2):
    return math.sqrt(
        math.pow(leg1.x - leg2.x, 2)
        + math.pow(leg1.y - leg2.y, 2)
        + math.pow(leg1.z - leg2.z, 2)
    )


def average(leg1, leg2):
    return Point(
        (leg1.x + leg2.x) / 2, (leg1.y + leg2.y) / 2, (leg1.z + leg2.z) / 2
    )


def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)


def subtract(v1, v2):
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)


def scale(v, s):
    v.x *= s
    v.y *= s
    v.z *= s


gen = MarkerGenerator()
gen.type = Marker.SPHERE
gen.frame_id = "base_link"
gen.lifetime = 0.5
gen.scale = [0.1, 0.1, 0.1]
gen.ns = "center"


# class KalmanFilter(object):
#     def __init__(self, pos, stamp):
#         self.pos = pos
#         self.stamp = stamp
#         self.reliability = 0.1
#         self.k = Kalman()

#     def update(self, pos):
#         last = self.pos

#         ivel = subtract(self.pos.pos, last.pos)
#         time = (self.pos.header.stamp - last.header.stamp).to_sec()
#         scale(ivel, 1.0 / time)

#         self.k.update([ivel.x, ivel.y, ivel.z])


class PersonAverageTracker(object):
    def __init__(self):
        self.kalman = Kalman()
        rospy.Subscriber("/people_cloud", PointCloud2, self.callback)
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )

        # self.kalman = KalmanFilter()

    def callback(self, msg):
        # print(msg.header.stamp)
        data = rn.point_cloud2.pointcloud2_to_xyz_array(msg)
        if data.shape[0] > 10:
            position = np.mean(data, axis=0)

            # pos_msg = PointStamped()
            # pos_msg.header = data.header
            # pos_msg.point.x = position[0]
            # pos_msg.point.y = position[1]
            # pos_msg.point.z = position[2]
            if position[0] <= 1.5:

                gen.counter = 0
                m = gen.marker(position=position)
                m.header = msg.header
                self.marker_pub.publish(m)


def main():
    rospy.init_node("people_average_tracker", anonymous=True)
    PersonAverageTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
