#!/usr/bin/env python

from contextlib import contextmanager

import matplotlib
import numpy as np
import seaborn as sns
from matplotlib import pyplot as plt

sns.set_theme()
matplotlib.use("Agg")


@contextmanager
def canvas(image_file=None, **kwargs):
    """Generic matplotlib context."""
    fig, ax = plt.subplots(**kwargs)
    # ax.grid(linestyle="dotted")
    # ax.set_aspect(1.0, "datalim")
    # ax.set_axisbelow(True)

    yield ax

    fig.set_tight_layout(True)
    if image_file:
        fig.savefig(image_file, dpi=300)
    # else:
    #     fig.show()
    plt.close(fig)


def annotate_xy(x_in, y_in, xy_text, ax=None):
    text = "{:.3f}m".format(y_in)
    if not ax:
        ax = plt.gca()
    bbox_props = dict(boxstyle="square,pad=0.1", fc="w", ec="k", lw=0.72)
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
        annotate_xy(x[idx], y[idx], xy_text=(x_text[i], y_text or 0.8), ax=ax)
