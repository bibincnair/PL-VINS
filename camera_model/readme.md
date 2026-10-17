# Camera models and calibration

This ament package exports the camera_model library with pinhole, equidistant,
Mei and Scaramuzza models. Build it as part of the workspace using the pinned
OpenCV 5, Eigen 5 and Ceres dependencies described in the repository README.

The calibration executable uses OpenCV command-line parsing and standard C++
filesystem support:

```bash
ros2 run camera_model camera_calibration --help
ros2 run camera_model camera_calibration calibrationdata --camera-model=pinhole
```

Images are processed without a GUI by default. Use --view-results=true with an
OpenCV build that has a GUI backend to view detected corners and reprojections.
