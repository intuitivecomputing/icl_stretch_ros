#!/usr/bin/env python3
import subprocess

import numpy as np
import ros_numpy as rn
import rosbag
import rospy
import typer

# player_proc = subprocess.Popen(['rosbag', 'play', bag_filename, ... (other args)], cwd=bagfile_folder_path)

def main(bag_name: str):
    with rosbag.Bag(bag_name, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/scan', 'sensor_msgs/LaserScan']):
            print(msg)


if __name__ == "__main__":
    typer.run(main)
