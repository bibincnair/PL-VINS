# Migration validation

Validated on Ubuntu 24.04 with ROS 2 Jazzy, using the pinned local dependency
build. All eight ament packages compile, including the calibration CLI, simulation
variants, custom line detector and standalone pose graph.

Completed checks:

- `colcon test-result --verbose`: 22 tests reported, zero errors/failures/skips.
  These include camera calibration loading and subpixel projection round trips,
  deterministic line descriptors, blank images, simulation row decoding, valid
  and degenerate relative-pose initialization, pose/line/yaw retraction identities,
  and finite-difference point/line/IMU/marginalization derivatives.
- Four Python tests: exact decimal-to-nanosecond conversion, rejected unordered
  input, image/IMU bag round trips, and rigid trajectory alignment without fitting
  scale. A separate ROS 1 bag conversion smoke check passed without ROS 1.
- Installed pipeline smoke: point, line and overlay output on 28 matching frames,
  pose-graph save through its ROS 2 service, and clean process shutdown.
- Recovered simulation: 589 finite poses from the installed combined publisher
  and estimator, with timestamp-preserving scheduling and clean shutdown.
- Installed ELF linkage audit: no missing libraries, ROS 1 libraries, or OpenCV
  3/4 dependencies in migrated executables/libraries.
- Recovered MH_05 feature/IMU recording: full playback at 5x speed, 1,092 finite
  output poses, clean shutdown. 1,080 poses overlap the bundled ground truth over
  107.9 seconds. Rigid SE(3) alignment, without scale fitting, gives position ATE
  RMSE **0.2764 m**, median **0.2511 m**, maximum **0.4740 m**.

Acceptance scripts explicitly isolate themselves on ROS domains 217 (pipeline),
218 (recording), and 219 (simulation), overriding inherited ROS_DOMAIN_ID.

Reproduce the recording test after recovering and converting the sample bag:

```bash
source tools/setup.bash
python3 tools/recover_data.py --prefix config/mh05_feature.bag --output data
python3 tools/recover_data.py --prefix benchmark_publisher/config/MH_05_difficult/ --output data
.venv/bin/python tools/dataset_bag.py convert-ros1 data/config/mh05_feature.bag --output data/mh05_ros2
.venv/bin/python tests/replay_features.py data/mh05_ros2 \
  --config config/euroc/euroc_config_fix_extrinsic.yaml --output results/mh05 --rate 5
.venv/bin/python tools/evaluate_trajectory.py results/mh05/trajectory_tum.txt \
  data/benchmark_publisher/config/MH_05_difficult/data.csv --output results/mh05/metrics.json
```

Numerical fixes exposed by the migration tests: analytic cost Jacobians now map
between manifold tangent and ambient coordinates; marginalization converts them
back before forming normal equations and handles quaternion sign consistently.
Initialization now uses upstream OpenCV pose recovery, rejecting empty models
instead of reshaping them; duplicated OpenCV symbols were removed. Shared
simulation readers accept both XYZUV and XYZWUV rows. The camera-frame line factor used a forward extrinsic rotation where its
translation derivative required the inverse; the corrected derivative agrees
with finite differences. Pose/line update conventions and state storage are
preserved. Camera inverse distortion retains its original eight iterations; the
round-trip check allows 0.05 pixels rather than assuming an exact inverse.

Limits: the MH_05 recording contains extracted features and IMU, rather than raw
images. The synthetic smoke covers image tracking and graph service startup, but
full MH_01/MH_05 raw-image loop-closure replay and comparison with an old build on
the same hardware remain unverified. Requests to the official raw-bag server
timed out in this environment. The reported ATE is not a legacy performance-parity
claim. Docker was unavailable locally; the Dockerfile and CI workflow have been
added, but the image itself has not been executed here. RViz was configured for
Jazzy but was not visually inspected in this headless environment.
