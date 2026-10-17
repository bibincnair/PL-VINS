#!/usr/bin/env python3
"""Plot one or more TUM trajectories (timestamp x y z qx qy qz qw)."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, default=Path('trajectory.png'))
    args = parser.parse_args()
    figure = plt.figure(); axes = figure.add_subplot(projection='3d')
    for path in args.trajectories:
        data = np.loadtxt(path, ndmin=2)
        if data.shape[1] < 4 or not np.isfinite(data[:, :4]).all(): parser.error(f'invalid trajectory: {path}')
        axes.plot(data[:, 1], data[:, 2], data[:, 3], label=path.stem)
    axes.set(xlabel='x [m]', ylabel='y [m]', zlabel='z [m]'); axes.legend()
    figure.savefig(args.output, dpi=160, bbox_inches='tight')

if __name__ == '__main__': main()
