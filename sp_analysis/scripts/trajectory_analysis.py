#!/usr/bin/env python
import logging
from pathlib import Path

import attr
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

from plot_helpers import annotates, canvas

LOG = logging.getLogger(__name__)


@attr.define
class TrajectoryAnalysis:
    smooth: bool = True
    plot: bool = False

    def __call__(self, file: str, output: str = None, **kwargs):
        file = Path(file)
        df = pd.read_json(file)
        df.columns = ["ts", "x", "y"]
        df["distance"] = df.iloc[:, 1:].apply(np.linalg.norm, axis=1)
        df["angle"] = np.arctan2(df["y"], df["x"])
        df["ts"] -= df["ts"][0]
        df["ts"] = pd.to_datetime(df["ts"])
        df.set_index("ts", inplace=True)
        df = df.sort_index()

        if self.smooth:
            df["distance_smoothed"] = df["distance"].rolling("3S").mean()

        if output is None:
            output = file.parent

            # find closest dist
        peaks, props = find_peaks(
            -df["distance_smoothed"],
            height=-1.1,
            distance=100,
            # width=3,
            # prominence=0.06,
        )

        if self.plot:

            LOG.info(f"Save figure to {Path(output) / (file.stem+'.png')}")
            with canvas(Path(output) / f"{file.stem}.png") as ax:
                df.plot(y="distance", ax=ax)
                if self.smooth:
                    df.plot(y="distance_smoothed", ax=ax)

                annotates(df.index, df["distance_smoothed"], peaks, ax=ax)

        return df
