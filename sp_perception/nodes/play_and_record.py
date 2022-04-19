#!/usr/bin/env python
import json
import sys
from pathlib import Path

import rosbag
import rospy
import tqdm
import typer
from tqdm.contrib.logging import logging_redirect_tqdm

from human_traj_analysis import PeakAnalysis
from leg_recorder import VelocityTracker

app = typer.Typer()


@app.command()
def main(name: str, bag: str, speed: float = 1):
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
            pub = rospy.Publisher(
                topic, type(msg), queue_size=10, latch=("map" in topic)
            )
            pubs[topic] = pub

        if t != last:
            # t = rospy.Time.from_sec(t.to_sec() / speed)  # speed up 4x
            data.append((t, []))
            last = t
        data[-1][1].append((topic, msg))
    rospy.loginfo("Done read")

    vt = VelocityTracker()
    vt.name = name

    start = rospy.Time.now()
    sim_start = None
    while not rospy.is_shutdown():
        for t, msgs in tqdm.tqdm(
            data, position=0, leave=False, desc=f"##{name}"
        ):
            now = rospy.Time.now()
            if sim_start is None:
                sim_start = t
            else:
                real_time = now - start
                sim_time = t - sim_start
                if sim_time > real_time:
                    rospy.sleep((sim_time - real_time))

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


@app.command()
def from_json(file: str):
    with open(file, "rb") as fp:
        trajectory = json.load(fp)
    print(f"Saving {Path(file).stem} to {Path(file).parent}")
    t, dist, peaks = PeakAnalysis(
        trajectory,
        output_dir=Path(file).parent,
        filename=Path(file).stem,
    )
    print(peaks)


@app.command()
def batch_json(folder: str):
    folder = Path(folder)
    for file in (
        pbar := tqdm.tqdm(
            sorted(list(folder.iterdir())), position=0, leave=False
        )
    ):
        if file.match("*.json"):
            from_json(str(file))
            # print(file)


@app.command()
def batch(folder: str):
    folder = Path(folder)
    # with typer.progressbar(folder.iterdir()) as progress:
    #     for subject in progress:
    for subject in (
        pbar := tqdm.tqdm(
            sorted(list(folder.iterdir())), position=0, leave=False
        )
    ):
        if subject.is_dir():
            pbar.set_description(f"#{subject.stem}")
            subject_id = subject.stem
            for i, bag in enumerate(sorted((subject / "raw").glob("*.bag"))):
                if bag.stem[0] == ".":
                    continue
                curr_id = subject_id + "-" + str(i + 1)
                with logging_redirect_tqdm():
                    main(name=curr_id, bag=bag)


if __name__ == "__main__":
    app()
