# PL-VINS on ROS 2 Jazzy

I ported PL-VINS to ROS 2 Jazzy and updated its build for Ubuntu 24.04 and
C++17. It estimates motion from point and line features, with optional loop
closure.

I kept the previous README here: [README.original.md](README.original.md).
For more detail, see [the architecture notes](docs/architecture.md) and
[LaMAria replay notes](docs/lamaria.md).

## What I changed

- I moved the runtime to ROS 2 packages, launch files, parameters, services,
  publishers, and subscribers. The estimator and feature tracker use ROS 2
  messages, simulated time, and sensor QoS.
- I pinned the updated build dependencies: OpenCV 5.0, Eigen 5.0, Ceres 2.2,
  SuiteSparse 7.14, Boost 1.92, glog 0.7, and `cv_bridge` 4.1. I also added
  compatibility patches for the newer OpenCV and Eigen APIs.

## Build

With ROS 2 Jazzy installed, build and set up the workspace:

```bash
bash tools/build.sh
source tools/setup.bash
```

Install the Python tools, including EVO, in the local environment:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --require-hashes -r requirements.lock -r requirements-evo.lock
```

## Run LaMAria

After the dataset is available under `/datasets/lamaraia`, run a sequence:

```bash
source tools/setup.bash
.venv/bin/python tools/run_lamaria.py \
  --dataset-root /datasets/lamaraia \
  --sequence R_01_easy \
  --output-root results/lamaria
```

Each run gets a new folder under the sequence, named with its UTC start time,
playback rate, and loop-closure setting. The runner keeps earlier runs and
saves the trajectories, ROS logs, EVO summaries, result archives, and plots.
Add `--run-label retry3` if you want a short label in the folder name. Use
`--loop-closure` to enable loop closure or `--no-evaluate` to skip EVO.

## Example results

This is one saved `R_01_easy` run at 0.5x playback with loop closure off. APE is
translation error after SE(3) alignment. RPE is measured over 1 m. These are
example results against LaMAria's dense pseudo-ground-truth, not the official
control-point benchmark score.

| Metric | Mean | RMSE | Plot |
|---|---:|---:|---|
| APE translation (m) | 0.969 | 1.022 | <img src="docs/images/lamaria-r01-ape-map.png" alt="APE mapped onto the trajectory" width="360"> |
| RPE translation per 1 m (m) | 1.378 | 1.429 | <img src="docs/images/lamaria-r01-rpe-translation.png" alt="Translation RPE over time" width="360"> |

The same run's rotation RPE over 1 m was 13.433° mean and 20.858° RMSE.

## License

This project is GPLv3; see [LICENCE](LICENCE). I retain the original research
citation in [support_files/paper_bib.txt](support_files/paper_bib.txt).
