#!/usr/bin/env python

import dataclasses
import json
import math
import pickle
from datetime import datetime
from pathlib import Path

import numpy as np
import ros_numpy as rnp
import rospy
from black import out
from easy_markers.generator import Marker, MarkerGenerator
from geometry_msgs.msg import Point, Vector3
from kalman_filter import Kalman
from people_msgs.msg import People, Person, PositionMeasurementArray
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


def printv(v):
    print(
        "%.2f %.2f %.2f" % (v.x, v.y, v.z),
    )


gen = MarkerGenerator()
gen.type = Marker.ARROW
gen.ns = "velocities"
gen.lifetime = 0.5


p_gen = MarkerGenerator()
p_gen.type = Marker.SPHERE
p_gen.frame_id = "base_link"
p_gen.lifetime = 0.5
p_gen.scale = [0.1, 0.1, 0.1]
p_gen.ns = "center"


class PersonEstimate(object):
    def __init__(self, msg):
        self.msg = msg
        self._reliability = 0.1
        self.k = Kalman()

    def update(self, msg):
        last = self.msg
        self.msg = msg
        self.reliability = msg.reliability
        ivel = subtract(self.position, last.pos)
        time = (self.timestamp - last.header.stamp).to_sec()
        scale(ivel, 1.0 / time)

        self.k.update([ivel.x, ivel.y, ivel.z])

    @property
    def timestamp(self):
        return self.msg.header.stamp

    @property
    def frame_id(self):
        return self.msg.header.frame_id

    @property
    def id(self):
        return self.msg.object_id

    @property
    def position(self):
        return self.msg.pos

    @property
    def reliability(self):
        return self._reliability

    @reliability.setter
    def reliability(self, value):
        self._reliability = max(self._reliability, value)

    @property
    def velocity(self):
        k = self.k.values()
        if k is None:
            return Vector3()
        v = Vector3(k[0], k[1], k[2])
        return v

    @property
    def person(self):
        p = Person()
        p.name = self.id
        p.position = self.position
        p.velocity = self.velocity
        p.reliability = self.reliability
        return p

    def publish_markers(self, pub):
        gen.scale = [0.1, 0.3, 0]
        gen.color = [1, 1, 1, 1]
        m = gen.marker(
            points=[self.position, add(self.position, self.velocity)]
        )
        m.header = self.msg.header
        pub.publish(m)

    def dump(self):
        return (
            # self.id,
            self.timestamp.to_sec(),
            self.position.x,
            self.position.y,
            self.velocity.x,
            self.velocity.y,
            self.reliability,
        )


class AverageTracker(object):
    def __init__(self, name: str = "000000"):
        self.name = name
        rospy.Subscriber("/people_cloud", PointCloud2, self.callback)
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )

        self.trajectory = []

    def callback(self, msg):
        # print(msg.header.stamp)
        data = rnp.point_cloud2.pointcloud2_to_xyz_array(msg)
        if data.shape[0] > 10:
            position = np.mean(data, axis=0)
            if position[0] <= 1.5:

                p_gen.counter = 0
                m = p_gen.marker(position=position)
                m.header = msg.header
                self.marker_pub.publish(m)

                self.trajectory.append(
                    [msg.header.stamp.to_nsec(), position[0], position[1]]
                )

    def dump(self, filename):
        with open(str(filename) + "_traj.json", "w") as fp:
            json.dump(self.trajectory, fp, indent=4)

    def on_exit(self):
        output_dir = (
            Path.home() / "catkin_ws" / "average_results" / f"{self.name}"
        )
        self.dump(output_dir)


class VelocityTracker(object):
    def __init__(self, plot=True):
        self.plot = plot
        self.name = rospy.get_param("~name", "000000")
        self.people = {}
        self.trajectory = []
        self.TIMEOUT = rospy.Duration(rospy.get_param("~timeout", 1.0))
        self.sub = rospy.Subscriber(
            "/people_tracker_measurements",
            PositionMeasurementArray,
            self.people_cb,
        )
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )
        self.people_pub = rospy.Publisher("/people", People, queue_size=10)

    def people_cb(self, msg):
        for person in msg.people:
            if person.object_id in self.people:
                self.people[person.object_id].update(person)
            else:
                p = PersonEstimate(person)
                self.people[person.object_id] = p

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.spin_once()
            rate.sleep()

        self.on_exit()

    def spin_once(self):
        # Remove People Older Than timeout param
        now = rospy.Time.now()
        for pid in list(self.people):
            if now - self.people[pid].timestamp > self.TIMEOUT:
                del self.people[pid]
        self.publish()
        self.record()

    def on_exit(self):

        output_dir = (
            Path.home() / "catkin_ws" / "study_results" / f"{self.name}"
        )
        self.dump(output_dir)

    def dump(self, filename):
        with open(str(filename) + ".json", "w") as fp:
            json.dump(self.trajectory, fp, indent=4)
        # with open(str(filename) + ".pkl", "wb") as fp:
        #     pickle.dump(self.trajectory, fp)

    def record(self):
        for p in self.people.values():
            self.trajectory.append(p.dump())

    def publish(self):
        gen.counter = 0
        msg = People()
        msg.header.frame_id = None

        if len(self.people) == 0:
            return

        for p in self.people.values():
            p.publish_markers(self.marker_pub)
            person = p.person
            msg.header.frame_id = p.frame_id
            msg.header.stamp = p.timestamp
            msg.people.append(person)

        self.people_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("people_velocity_tracker")
    vt = VelocityTracker()
    vt.spin()
