#!/usr/bin/env python
import json
import sys
from pathlib import Path

import rosbag
import rospy
import tqdm
import typer
from tqdm.contrib.logging import logging_redirect_tqdm

from .tracking import AverageTracker, VelocityTracker

app = typer.Typer()


def process(name: str, bag: str, speed: float = 1, start_time: float = 0):
    rospy.init_node("play_and_record", log_level=rospy.DEBUG)
    bagfn = bag
    should_loop = False
    loop_sleep = 0.1

    if bagfn is None:
        rospy.logerr("No Bag specified!")
        exit(1)

    pubs = {}
    rospy.loginfo("Start read")
    last = None
    data = []
    bag = None
    start = None

    try:
        with rosbag.Bag(bagfn) as bag:
            for topic, msg, t in bag.read_messages():
                if topic not in pubs:
                    pub = rospy.Publisher(
                        topic, type(msg), queue_size=10, latch=("map" in topic)
                    )
                    pubs[topic] = pub

                if start is None:
                    start = t
                if start_time != 0:
                    time_passed = t - start
                    print(f"Skipped:{time_passed.secs}s")
                    if time_passed.secs <= start_time:
                        continue
                    else:
                        start_time = 0
                if t != last:
                    # t = rospy.Time.from_sec(t.to_sec() / speed)  # speed up 4x
                    data.append((t, []))
                    last = t
                data[-1][1].append((topic, msg))
    except Exception as e:
        rospy.logerr(f"Exp: {e} for {bagfn}")
        # raise e
        return

    rospy.loginfo("Done read")

    # velocity_tracker = VelocityTracker(plot=False)
    # velocity_tracker.name = name
    average_tracker = AverageTracker(name)

    start = rospy.Time.now()
    sim_start = None
    while not rospy.is_shutdown():
        for t, msgs in tqdm.tqdm(
            data, position=0, leave=True, desc=f"##{name}"
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
            # velocity_tracker.spin_once()
        if not should_loop:
            # rospy.signal_shutdown("rospy shutdown")
            break

        rospy.sleep(loop_sleep)
    # velocity_tracker.on_exit()
    average_tracker.on_exit()


@app.command()
def main(name: str, bag: str, speed: float = 1, start_time: float = 0):
    process(name, bag, speed=speed, start_time=start_time)


# @app.command()
# def from_json(file: str):
#     with open(file, "rb") as fp:
#         trajectory = json.load(fp)
#     print(f"Saving {Path(file).stem} to {Path(file).parent}")
#     # print(trajectory)
#     t, dist, peaks = PeakAnalysis(
#         trajectory,
#         output_dir=Path(file).parent,
#         filename=Path(file).stem,
#     )
#     print(peaks)


# @app.command()
# def batch_json(folder: str):
#     folder = Path(folder)
#     for file in (
#         pbar := tqdm.tqdm(
#             sorted(list(folder.iterdir())), position=0, leave=True
#         )
#     ):
#         if file.match("*.json"):
#             try:
#                 from_json(str(file))
#             # print(file)
#             except Exception as e:
#                 print(e)


@app.command()
def batch(folder: str, start_from: str = None, user_id: str = None):
    folder = Path(folder)
    # with typer.progressbar(folder.iterdir()) as progress:
    #     for subject in progress:

    subjects = sorted(list(folder.iterdir()))
    subject_ids = [subject.stem for subject in subjects]
    if start_from is not None:
        if start_from in subject_ids:
            start_from_index = subject_ids.index(start_from)
            rospy.loginfo(
                f"Starting from subject {start_from}, index {start_from_index}."
            )
            subjects = subjects[start_from_index:]
    elif user_id is not None:
        if user_id in subject_ids:
            index = subject_ids.index(user_id)
            rospy.loginfo(f"Processing subject {user_id}, index {index}.")
            subjects = [subjects[index]]

    for subject in (pbar := tqdm.tqdm(subjects, position=0, leave=True)):
        if subject.is_dir():
            pbar.set_description(f"#{subject.stem}")
            subject_id = subject.stem
            for i, bag in enumerate(
                sorted((subject / "raw").glob("[!.]*.bag"))
            ):
                # print(bag)
                if bag.stem[0] == ".":
                    continue

                curr_id = subject_id + "-" + str(i + 1)
                with logging_redirect_tqdm():
                    # try:
                    process(name=curr_id, bag=bag)
                # except KeyboardInterrupt:
                #     raise typer.Exit()
                # except Exception as e:
                #     raise e


if __name__ == "__main__":
    app()
