#!/usr/bin/env python
import json
import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import typer

LOG = logging.getLogger(__name__)


@dataclass
class VideoSource:
    fid: int
    cam: str  # cam1, cam2, gaze
    task: int  # 1,2

    @classmethod
    def from_dict(cls, data: dict):
        print(data)
        fid = int(data["fid"])
        fname = data["fname"].split(".")[0]
        split_char = "_" if "_" in fname else "-"
        cam = fname.split(split_char)[0]
        task = int(fname.split(split_char)[1][-1])
        LOG.info(f"fid:{fid}, cam:{cam}, task:{task}")
        return cls(fid, cam, task)


def main(file: str):
    file = Path(file)
    try:
        with open(file, "rb") as fp:
            data = json.load(fp)
    except Exception as err:
        LOG.error(err)
        return
    print(data.keys())
    for _, item in data["file"].items():
        v = VideoSource.from_dict(item)


if __name__ == "__main__":
    typer.run(main)
