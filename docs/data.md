# External research data

Historical trajectories, sample bags, benchmark ground truth and simulation data
are no longer part of the source checkout. Every removed asset is recorded with
a SHA-256 checksum and its pre-migration Git revision in `data-manifest.json`.
Git history was not rewritten. Recover data from a clone containing that revision:

```bash
python3 tools/recover_data.py --prefix config/simdata/data/ --output data
python3 tools/recover_data.py --prefix benchmark_publisher/config/ --output data
python3 tools/recover_data.py --prefix Trajactory/ --output data
```

Recovery never overwrites an existing file. If the repository was cloned shallowly,
fetch the revision named in the manifest first. The manifest also allows verifying
copies obtained separately. In this migration workspace an additional verified
archive was saved at `/tmp/plvins-research-assets-4292f04fd7c0.tar.gz`; reproducible
recovery relies on Git, not on that temporary archive.

Download new EuRoC sequences from the [official dataset site](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
Use the ROS 2 bag tools below to import a raw `mav0` directory:

```bash
source tools/setup.bash
.venv/bin/python tools/dataset_bag.py create \
  --image-csv data/MH_01_easy/mav0/cam0/data.csv \
  --image-root data/MH_01_easy/mav0/cam0/data \
  --imu-csv data/MH_01_easy/mav0/imu0/data.csv \
  --groundtruth-csv data/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  --output data/MH_01_easy_ros2
```

CSV timestamps default to nanoseconds; `--timestamp-unit seconds` accepts exact
decimal second timestamps. Image rows are `timestamp,filename`; IMU rows are
`timestamp,wx,wy,wz,ax,ay,az`; ground-truth rows are
`timestamp,px,py,pz,qw,qx,qy,qz[,vx,vy,vz,...]`. Lines starting with `#` are skipped.
The same normalized format supports other datasets without dataset-specific ROS 1
scripts. Ground truth is odometry, rather than being mislabeled as IMU data.

```bash
.venv/bin/python tools/dataset_bag.py convert-ros1 data/input.bag --output data/input_ros2
.venv/bin/python tools/dataset_bag.py extract data/input_ros2 --output data/images
```

Conversion uses the pure Python `rosbags` package and targets the Jazzy type store.
No ROS 1 installation or runtime bridge is required. Tools refuse existing output
bags/directories rather than overwriting them. Outputs use MCAP; extraction also
accepts other formats supported by the installed rosbag2 storage plugins.
