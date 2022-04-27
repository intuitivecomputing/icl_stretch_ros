#!/usr/bin/env python
import json
import pickle
from contextlib import contextmanager
from copyreg import pickle
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np
import typer
from matplotlib import pyplot as plt

matplotlib.use("Agg")
from scipy.signal import find_peaks, find_peaks_cwt


@contextmanager
def canvas(image_file=None, **kwargs):
    """Generic matplotlib context."""
    fig, ax = plt.subplots(**kwargs)
    ax.grid(linestyle="dotted")
    # ax.set_aspect(1.0, "datalim")
    # ax.set_axisbelow(True)

    yield ax

    fig.set_tight_layout(True)
    if image_file:
        fig.savefig(image_file, dpi=300)
    # fig.show()
    plt.close(fig)


def annotate_xy(x_in, y_in, xy_text, ax=None):
    text = "{:.3f}m".format(y_in)
    if not ax:
        ax = plt.gca()
    bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="k", lw=0.72)
    arrowprops = dict(arrowstyle="->", connectionstyle="arc")
    kw = dict(
        xycoords="data",
        textcoords="axes fraction",
        arrowprops=arrowprops,
        bbox=bbox_props,
        ha="right",
        va="top",
    )
    ax.annotate(text, xy=(x_in, y_in), xytext=xy_text, **kw)


def annotates(x, y, indices, ax=None, y_text=None):
    plt.plot(x[indices], y[indices], "x")
    x_text = np.linspace(0, 1, len(indices))
    for i, idx in enumerate(indices):
        annotate_xy(x[idx], y[idx], xy_text=(x_text[i], y_text or 0.3), ax=ax)


def find_closest_dist(data: np.ndarray):
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    # vel_x = data[:, 3]
    # vel_y = data[:, 4]
    dist = np.sqrt(data[:, 1] ** 2 + data[:, 2] ** 2)
    angle = np.rad2deg(np.arctan2(data[:, 2], data[:, 1]))
    # find closest dist
    peaks, props = find_peaks(
        -dist, height=-1, distance=300, width=10, prominence=0.08
    )
    return t, dist, peaks


def PeakAnalysis(data, output_dir, filename):
    # pos = np.asarray([datum[1:] for datum in data])
    data = np.asarray(data)
    t, dist, peaks = find_closest_dist(data)
    t = t / 1000000000  # s
    # plot dist
    with canvas(Path(output_dir) / f"{filename}.png") as ax:
        ax.plot(t, dist)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (m)")
        annotates(t, dist, peaks, ax=ax)
    return t, dist, peaks

    # with canvas(
    #     Path(output_dir)
    #     / f"results-{datetime.now().strftime('%m%d%H%M')}-traj.png"
    # ) as ax:
    #     # ax.plot(x, y, marker="o", markevery=5)
    #     ax.plot(
    #         x,
    #         y,
    #     )


app = typer.Typer()


@app.command()
def main(file: str):
    file = Path.home() / "catkin_ws" / "study_results" / (file + ".json")
    with open(file, "rb") as f:
        data = json.load(f)
    PeakAnalysis(data, file.parent, file.stem)


@app.command()
def batch(folder: str):
    data = {}
    folder = Path(folder)
    file_list = sorted(folder.glob("*.json"))
    file_list = [f.stem for f in file_list]
    print(f"Processing {file_list}")
    id_list = set([f.split("-")[0] for f in file_list])
    colors = ["red", "green", "blue"]

    with typer.progressbar(id_list) as progress:
        for id in progress:
            typer.echo(f"Processing participant #{id}")
            with canvas(folder / id) as ax:
                ax.set_xlabel("Time (nsec)")
                ax.set_ylabel("Distance (m)")
                id_file_list = sorted([f for f in file_list if id in f])
                for i, file in enumerate(id_file_list):
                    with open(folder / (file + ".json"), "rb") as f:
                        data = json.load(f)
                    data = np.asarray(data)
                    t, dist, peaks = find_closest_dist(data)
                    t = t - t[0]
                    ax.plot(t, dist, color=colors[i])
                    annotates(t, dist, peaks, ax=ax, y_text=0.1 * (i + 1))


if __name__ == "__main__":
    app()
