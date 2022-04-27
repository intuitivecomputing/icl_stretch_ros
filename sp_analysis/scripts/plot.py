#!/usr/bin/env python
import json
import logging
import sys
from pathlib import Path

import rosbag
import rospy
import tqdm
import typer
from tqdm.contrib.logging import logging_redirect_tqdm

from human_traj_analysis import PeakAnalysis

LOG = logging.getLogger(__name__)

app = typer.Typer()


@app.command()
def main(file: str):
    with open(file, "rb") as fp:
        trajectory = json.load(fp)

    LOG.info(f"Saving {Path(file).stem}.png to {Path(file).parent}")
    # print(trajectory)
    try:
        t, dist, peaks = PeakAnalysis(
            trajectory,
            output_dir=Path(file).parent,
            filename=Path(file).stem,
        )
        # print(peaks)
    except:
        # ignore
        return


@app.command()
def batch_json(folder: str):
    folder = Path(folder)
    for file in (pbar := tqdm.tqdm(sorted(list(folder.iterdir())))):
        if file.match("*.json"):
            with logging_redirect_tqdm():
                main(str(file))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app()
