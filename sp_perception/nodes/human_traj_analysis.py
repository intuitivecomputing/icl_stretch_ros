from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import find_peaks


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


def annotates(x, y, indices, ax=None):
    plt.plot(x[indices], y[indices], "x")
    x_text = np.linspace(0, 1, len(indices))
    for i, idx in enumerate(indices):
        annotate_xy(x[idx], y[idx], xy_text=(x_text[i], 0.3), ax=ax)


def PeakAnalysis(data, output_dir):
    pos = np.asarray([datum[1:] for datum in data])
    t = pos[:, 0]
    x = pos[:, 1]
    y = pos[:, 2]
    vel_x = pos[:, 3]
    vel_y = pos[:, 4]

    dist = np.sqrt(pos[:, 1] ** 2 + pos[:, 2] ** 2)
    angle = np.rad2deg(np.arctan2(pos[:, 2], pos[:, 1]))
    # find closest dist
    peaks, props = find_peaks(-dist, height=-1, width=20, prominence=0.1)

    # plot dist
    with canvas(
        Path(output_dir) / f"results-{datetime.now().strftime('%m%d%H%M')}.png"
    ) as ax:
        ax.plot(t, dist)
        ax.set_xlabel("Time (nsec)")
        ax.set_ylabel("Distance (m)")
        annotates(t, dist, peaks, ax=ax)

    # with canvas(
    #     Path(output_dir)
    #     / f"results-{datetime.now().strftime('%m%d%H%M')}-traj.png"
    # ) as ax:
    #     # ax.plot(x, y, marker="o", markevery=5)
    #     ax.plot(
    #         x,
    #         y,
    #     )
