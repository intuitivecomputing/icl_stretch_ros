#!/usr/bin/env python3
import numpy as np
import ros_numpy as rn
import rospy
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import PointCloud2


def callback(msg):
    data = rn.numpify(msg)
    if data.shape[0] !=0:
        print(data.shape)


def main():
    rospy.init_node("people_logger", anonymous=True)
    rospy.Subscriber("/people_cloud", PointCloud2, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
