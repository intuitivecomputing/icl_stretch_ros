#!/usr/bin/env python
import sys

import rosbag
import rospy
import typer

from leg_recorder import VelocityTracker


def main(name: str, bag: str):
    rospy.init_node("play_and_record")
    bagfn = bag
    should_loop = False
    loop_sleep = 0.1

    if bagfn is None:
        rospy.logerr("No Bag specified!")
        exit(1)

    bag = rosbag.Bag(bagfn)
    pubs = {}
    rospy.loginfo("Start read")
    last = None
    data = []

    for topic, msg, t in bag.read_messages():
        if topic not in pubs:
            pub = rospy.Publisher(topic, type(msg), latch=("map" in topic))
            pubs[topic] = pub

        if t != last:
            data.append((t, []))
            last = t
        data[-1][1].append((topic, msg))
    rospy.loginfo("Done read")

    vt = VelocityTracker()
    vt.name = name

    start = rospy.Time.now()
    sim_start = None
    while not rospy.is_shutdown():
        for t, msgs in data:
            now = rospy.Time.now()
            if sim_start is None:
                sim_start = t
            else:
                real_time = now - start
                sim_time = t - sim_start
                if sim_time > real_time:
                    rospy.sleep(sim_time - real_time)

            for (topic, msg) in msgs:
                if "header" in dir(msg):
                    msg.header.stamp = now
                elif "transforms" in dir(msg):
                    for tf in msg.transforms:
                        tf.header.stamp = now
                pub = pubs[topic]
                pub.publish(msg)
            if rospy.is_shutdown():
                break
            vt.spin_once()
        if not should_loop:
            break

        rospy.sleep(loop_sleep)
    vt.on_exit()
    bag.close()
