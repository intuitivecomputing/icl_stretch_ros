#!/usr/bin/env python
import logging
from pathlib import Path

import attrs
import matplotlib.dates as mdates
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

from .plot_helpers import annotates, canvas

LOG = logging.getLogger(__name__)


@attrs.define
class TrajectoryAnalysis:
    smooth: bool = True
    plot: bool = False
    plot_2d: bool = False

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
            df["distance_smoothed"] = df["distance"].rolling(window="3S").mean()
        else:
            df["distance_smoothed"] = df["distance"]

        if output is None:
            output = file.parent

        # find closest dist
        peaks, props = find_peaks(
            -df["distance_smoothed"],
            height=-1.5,
            distance=50,
            width=3,
            # prominence=0.06,
        )

        if self.plot:

            LOG.info(f"Save figure to {Path(output) / (file.stem+'.png')}")
            with canvas(Path(output) / f"{file.stem}.png") as ax:
                locator = mdates.AutoDateLocator()
                formatter = mdates.DateFormatter("%M:%S")
                # formatter = mdates.ConciseDateFormatter(locator)
                ax.xaxis.set_major_locator(locator)
                ax.xaxis.set_major_formatter(formatter)

                df.plot(y="distance", ax=ax)
                if self.smooth:
                    df.plot(y="distance_smoothed", ax=ax)

                annotates(df.index, df["distance_smoothed"], peaks, ax=ax)
                ax.set_xlabel("Time")
                ax.set_ylabel("Distance (m)")

        if self.plot_2d:
            LOG.info(f"Save figure to {Path(output) / (file.stem+'_2d.png')}")
            with canvas(Path(output) / f"{file.stem}_2d.png") as ax:
                df.plot.scatter(x="x", y="y", ax=ax)
                ax.set_xlabel("X (m)")
                ax.set_ylabel("Y (m)")

        return df
