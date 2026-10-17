#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include "line_descriptor_custom.hpp"

TEST(LineDescriptor,DetectsAndDescribesSyntheticLines) {
  cv::Mat image(480,752,CV_8UC1,cv::Scalar(255));
  for (int x=50;x<700;x+=80) cv::line(image,{x,30},{x+30,450},cv::Scalar(0),3);
  auto detector=cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
  cv::line_descriptor::LSDDetectorC::LSDOptions options;
  options.refine=1;options.scale=.5;options.sigma_scale=.6;options.quant=2.;
  options.ang_th=22.5;options.log_eps=1.;options.density_th=.6;options.n_bins=1024;options.min_length=60.;
  std::vector<cv::line_descriptor::KeyLine> lines;
  detector->detect(image,lines,2,1,options);
  ASSERT_GT(lines.size(),0u);
  auto descriptor=cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
  cv::Mat first,second;
  descriptor->compute(image,lines,first); descriptor->compute(image,lines,second);
  ASSERT_EQ(first.rows,static_cast<int>(lines.size()));
  ASSERT_EQ(first.cols,32);
  EXPECT_EQ(cv::norm(first,second,cv::NORM_HAMMING),0.);
}
TEST(LineDescriptor,BlankImageHasNoLines) {
  auto detector=cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
  std::vector<cv::line_descriptor::KeyLine> lines;
  detector->detect(cv::Mat(480,752,CV_8UC1,cv::Scalar(0)),lines,2,1);
  EXPECT_TRUE(lines.empty());
}
