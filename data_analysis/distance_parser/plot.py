#!/usr/bin/env python
import json
import logging
from pathlib import Path

import tqdm
import typer
from tqdm.contrib.logging import logging_redirect_tqdm

from trajectory_analysis import TrajectoryAnalysis

LOG = logging.getLogger(__name__)

app = typer.Typer()


@app.command()
def main(file: str, plot: bool = True, plot_2d: bool = False):
    analysis = TrajectoryAnalysis(plot=plot, plot_2d=plot_2d)
    try:
        analysis(file)
    except Exception as e:
        LOG.warning(f"Parsing {Path(file).stem} failed with [{e}]")
        return


@app.command()
def batch_json(folder: str):
    folder = Path(folder)
    for file in (pbar := tqdm.tqdm(sorted(list(folder.glob("[!.]*.json"))))):
        with logging_redirect_tqdm():
            main(str(file))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app()
