#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"

TEST(CameraModel,CalibrationLoadsAndProjectionRoundTrips) {
  auto camera=camodocal::CameraFactory::instance()->generateCameraFromYamlFile(PLVINS_TEST_CALIBRATION);
  ASSERT_TRUE(camera);
  for (const Eigen::Vector2d& pixel: {Eigen::Vector2d(100.,120.),Eigen::Vector2d(376.,240.),Eigen::Vector2d(650.,400.)}) {
    Eigen::Vector3d ray; Eigen::Vector2d result;
    camera->liftProjective(pixel,ray); camera->spaceToPlane(ray,result);
    // The original model uses eight fixed-point inverse-distortion iterations.
    EXPECT_LT((result-pixel).norm(),.05);
  }
  cv::Mat map1,map2;
  const auto K=camera->initUndistortRectifyMap(map1,map2);
  EXPECT_EQ(map1.rows,480); EXPECT_EQ(map1.cols,752); EXPECT_EQ(K.rows,3);
}
TEST(CameraModel,MissingCalibrationIsRejected) {
  EXPECT_FALSE(camodocal::CameraFactory::instance()->generateCameraFromYamlFile("/nonexistent/plvins-calibration.yaml"));
}
