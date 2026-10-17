# PL-VINS for ROS 2 Jazzy

Monocular visual-inertial SLAM using point and line features, with optional
BRIEF/DBoW loop closure. This migration targets Ubuntu 24.04, ROS 2 Jazzy and
C++17. See [architecture](docs/architecture.md) for the processing pipeline and
[external data](docs/data.md) for datasets and historical result recovery.

## Build

Install ROS 2 Jazzy and the build prerequisites listed in `Dockerfile`, then run:

```bash
bash tools/build.sh
source tools/setup.bash
```

The build downloads checksum-verified sources pinned in `dependencies.lock.json`:
OpenCV 5.0.0, Eigen 5.0.1, Ceres 2.2.0, SuiteSparse 7.14.1, Boost 1.92.0 headers,
and glog 0.7.1. The Jazzy `cv_bridge` 4.1.0 C++ library is rebuilt against OpenCV 5.
Compatibility changes are applied by the bootstrap script and recorded in the
lock file. Dependencies install locally under `.deps/install`; no `sudo make
install` or developer-specific paths are required. Set `PLVINS_JOBS` to control
compiler concurrency.

Alternatively, `docker build -t plvins:jazzy .` builds the same source tree and
pinned libraries. Mount external datasets and a writable result directory when
running the container. Container GUI forwarding is optional.

Python tools use a separate environment with pinned, hashed dependencies:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --require-hashes -r requirements.lock
```

Source `tools/setup.bash` before using ROS Python tools. Python OpenCV and the
Python `cv_bridge` extension are not required.

## Run

```bash
ros2 launch plvins_estimator plvins.launch.py \
  use_sim_time:=true rviz:=true overlay:=true output_directory:=/tmp/plvins-results
ros2 bag play data/MH_01_easy_ros2 --clock
```

The default camera calibration is EuRoC. Select another installed or external
OpenCV YAML using `config_file:=/absolute/path/to/calibration.yaml`. Use
`loop_closure:=false` to disable the pose graph. For a live camera, leave
`use_sim_time:=false` and configure the camera and IMU topics in the calibration
file. Sensor inputs use best-effort QoS; internal feature messages use reliable
QoS. See [architecture](docs/architecture.md) for feature channel layouts and
[LaMAria replay and EVO metrics](docs/lamaria.md) for the installed ROS 2 bags.

The estimator writes `trajectory_tum.txt` (seconds, xyz, qxyzw) and
`trajectory_ns.txt` (nanoseconds) into `output_directory`. The pose graph writes
its loop trajectories there and saves its graph under `pose_graph/` by default.
Override the latter with `pose_graph_directory` when loading an existing graph.

```bash
ros2 service call /pose_graph/save_graph std_srvs/srv/Trigger '{}'
ros2 service call /pose_graph/new_sequence std_srvs/srv/Trigger '{}'
ros2 launch plvins_estimator simulation.launch.py \
  data_directory:=$PWD/data/config/simdata/data
ros2 launch benchmark_publisher publish.launch.py \
  data_name:=$PWD/data/benchmark_publisher/config/MH_01_easy/data.csv
ros2 run camera_model camera_calibration --help
.venv/bin/python pythontool/draw_trajcory.py results/trajectory_tum.txt --output trajectory.png
```

## Validate

```bash
source tools/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 tools/check_linkage.py
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```

Run `.venv/bin/python tests/smoke_ros2.py` for the installed multi-node smoke test.
See [recorded validation and limits](docs/validation.md) for the MH_05 replay results.

C++ tests check manifold identities and finite-difference point, line, IMU and
marginalization derivatives. Python tests check timestamp precision and bag image
round trips. Full accuracy/performance acceptance additionally requires replaying
EuRoC MH_01 and MH_05 with loop closure enabled and disabled on the same hardware
as a recorded legacy baseline; historical trajectories alone are not a timing
baseline.

## Research and license

Based on [PL-VINS](https://arxiv.org/abs/2009.07462), PL-VIO and VINS-Mono.
Please cite the original research when using this system. Original citation
metadata is retained in `support_files/paper_bib.txt`. Project code is GPLv3;
see `LICENCE` and third-party source notices for component licenses.
