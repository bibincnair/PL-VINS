#include <gtest/gtest.h>
#include "../src/dataset_io.h"
#include <filesystem>
#include <unistd.h>
namespace {
struct File {
  std::filesystem::path path=std::filesystem::temp_directory_path()/("plvins-dataset-test-"+std::to_string(getpid())+".txt");
  explicit File(const std::string& text) { std::ofstream(path)<<text; }
  ~File() { std::filesystem::remove(path); }
};
}
TEST(SimulationIo,HomogeneousWorldPointDoesNotBecomeImageCoordinate) {
  File file("0 0 6.5 1 .2 -.3\n0 0 6.5 .4 -.5\n.6 -.7\n");
  std::vector<Eigen::Vector2d> observations;
  plvins::simulation::LoadPointObs(file.path.string(),observations);
  ASSERT_EQ(observations.size(),3u);
  EXPECT_DOUBLE_EQ(observations[0].x(),.2); EXPECT_DOUBLE_EQ(observations[0].y(),-.3);
  EXPECT_DOUBLE_EQ(observations[1].x(),.4); EXPECT_DOUBLE_EQ(observations[2].x(),.6);
}
TEST(SimulationIo,MalformedPointRowsAreRejected) {
  File file("1 2 3\n");std::vector<Eigen::Vector2d> observations;
  EXPECT_THROW(plvins::simulation::LoadPointObs(file.path.string(),observations),std::runtime_error);
}
TEST(SimulationIo,PoseTimingAndFieldsArePreserved) {
  File file("0.005 1 0 0 0 1 2 3 .1 .2 .3 0 0 9.81\n");
  std::vector<double> timestamps;std::vector<Eigen::Vector3d> gyros,accelerations;
  plvins::simulation::LoadPose(file.path.string(),timestamps,gyros,accelerations);
  ASSERT_EQ(timestamps.size(),1u);EXPECT_DOUBLE_EQ(timestamps[0],.005);
  EXPECT_DOUBLE_EQ(gyros[0].z(),.3);EXPECT_DOUBLE_EQ(accelerations[0].z(),9.81);
}
