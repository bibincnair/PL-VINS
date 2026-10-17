# Replaying LaMAria bags

LaMAria's [ROS bag and calibration documentation](https://lamaria.inf.ethz.ch/slam_documentation)
defines the sensor topics, calibration transforms, ground-truth pose frame, and
timestamp conventions used below. EVO's [format guide](https://github.com/MichaelGrupp/evo/wiki/Formats)
and [metrics guide](https://github.com/MichaelGrupp/evo/wiki/Metrics) describe its
TUM input format and alignment options.

The mounted data is at `/datasets/lamaraia` (the directory name has an extra
`a`). It contains `R_01_easy`, `R_04_medium`, and `R_08_hard`. Each sequence has
a Jazzy MCAP bag, a pinhole calibration JSON, and a dense pseudo-ground-truth
trajectory. The bags publish `/cam0/image_raw`, `/cam1/image_raw`, and `/imu0`;
this PL-VINS run uses the left monocular camera and IMU.

Build and source the workspace first. Install EVO into the project Python
environment when trajectory metrics are wanted:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --require-hashes -r requirements.lock -r requirements-evo.lock
source tools/setup.bash
```

For live Foxglove visualization, install the ROS bridge once:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Run a sequence and calculate metrics:

```bash
.venv/bin/python tools/run_lamaria.py \
  --dataset-root /datasets/lamaraia \
  --sequence R_01_easy \
  --output-root results/lamaria
```

Repeat with `R_04_medium` or `R_08_hard`. The runner reads each sequence's
calibration JSON, writes an OpenCV config with its cam0 intrinsics and
IMU-to-camera extrinsics, starts `plvins.launch.py` using simulated time, waits
for all three estimator nodes, and plays the MCAP with `/clock`. It then
converts the LaMAria nanosecond ground truth into EVO's TUM seconds format and
converts PL-VINS world-from-IMU estimates to world-from-cam0 using the same
calibration before comparing poses. The generated tracker config publishes on
every input image so estimates remain aligned with the sequence timestamps.

Each run gets a new directory such as `results/lamaria/R_01_easy/`. It contains
the generated config, ROS launch and playback logs, raw estimator trajectory,
camera-frame estimate, converted reference, and EVO APE/RPE text plus result
archives. It also writes `<sequence>.txt` with nanosecond timestamps and
world-from-IMU poses in LaMAria's VIO estimate format. Existing output
directories are never overwritten. Playback defaults to `--playback-rate 0.5`;
adjust this on a constrained machine,
`--loop-closure` to run the pose graph, or `--no-evaluate` to skip EVO.

Foxglove runs separately from the PL-VINS launch. In another terminal, run
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765` and connect
to `ws://localhost:8765`. The `.mcap` files can also be opened directly in
Foxglove for offline inspection.

The runner reports EVO APE after rigid SE(3) alignment and translational and
rotational RPE over 1 m. APE is the useful controlled-sequence comparison; the
RPE values are additional local-drift diagnostics. LaMAria documents dense pGT
as pseudo ground truth and its controlled experimental set's ATE after Umeyama
alignment. The main benchmark also offers control-point evaluation; EVO results
here should not be presented as the full sparse-control-point benchmark score.
The provided text trajectories are in the format expected by LaMAria, but a
submission should additionally provide all required timestamps and use the
official evaluation process.

For manual replay after generating a config with the runner, start PL-VINS in
one terminal and the bag in another:

```bash
ros2 launch plvins_estimator plvins.launch.py \
  config_file:=results/lamaria/R_01_easy/config.yaml \
  output_directory:=results/lamaria/R_01_easy use_sim_time:=true
ros2 bag play /datasets/lamaraia/R_01_easy --clock
```

Run the Foxglove bridge command in a separate terminal when the bridge package
is installed.
