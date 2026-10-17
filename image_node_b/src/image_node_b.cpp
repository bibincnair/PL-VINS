#include <plvins_common/ros.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv2/imgproc.hpp>
#include "camodocal/camera_models/CameraFactory.h"

class FeatureOverlay final : public rclcpp::Node {
  using Cloud = sensor_msgs::msg::PointCloud;
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ExactTime<Cloud,Cloud,Image>;
 public:
  FeatureOverlay(): Node("image_node_b") {
    const auto config=declare_parameter<std::string>("config_file", "");
    cv::FileStorage settings(config,cv::FileStorage::READ);
    if (!settings.isOpened()) throw std::runtime_error("Cannot open config_file: " + config);
    std::string topic; settings["image_topic"] >> topic;
    auto camera=camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config);
    if (!camera) throw std::runtime_error("Unsupported camera model");
    K_=camera->initUndistortRectifyMap(map1_,map2_);
    points_.subscribe(this,"/feature_tracker/feature",rclcpp::QoS(20).get_rmw_qos_profile());
    lines_.subscribe(this,"/linefeature_tracker/linefeature",rclcpp::QoS(20).get_rmw_qos_profile());
    images_.subscribe(this,topic,rmw_qos_profile_sensor_data);
    publisher_=create_publisher<Image>("/PointLine_image",10);
    synchronizer_=std::make_unique<message_filters::Synchronizer<Policy>>(Policy(20),points_,lines_,images_);
    synchronizer_->registerCallback(&FeatureOverlay::synchronized, this);
  }
 private:
  void synchronized(Cloud::ConstSharedPtr points, Cloud::ConstSharedPtr lines, Image::ConstSharedPtr image) { render(*points,*lines,image); }
  bool valid(const Cloud& cloud) {
    if (cloud.channels.size()<3) return cloud.points.empty();
    for (size_t c=0;c<3;++c) if (cloud.channels[c].values.size()!=cloud.points.size()) return false;
    return true;
  }
  void render(const Cloud& points,const Cloud& lines,Image::ConstSharedPtr image) {
    if (!valid(points) || !valid(lines)) { RCLCPP_WARN(get_logger(),"Malformed feature frame"); return; }
    try {
      auto input=cv_bridge::toCvCopy(image,"bgr8");
      for (size_t i=0;i<points.points.size();++i) {
        const float u=points.channels[1].values[i],v=points.channels[2].values[i];
        if (std::isfinite(u) && std::isfinite(v)) cv::circle(input->image,cv::Point2f(u,v),2,cv::Scalar(0,255,0),2);
      }
      cv::Mat output; cv::remap(input->image,output,map1_,map2_,cv::INTER_LINEAR);
      auto pixel=[&](double x,double y) { return cv::Point2f(x*K_.at<float>(0,0)+K_.at<float>(0,2),y*K_.at<float>(1,1)+K_.at<float>(1,2)); };
      for (size_t i=0;i<lines.points.size();++i) {
        const auto& p=lines.points[i]; const float x=lines.channels[1].values[i],y=lines.channels[2].values[i];
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(x) && std::isfinite(y))
          cv::line(output,pixel(p.x,p.y),pixel(x,y),cv::Scalar(0,0,255),2);
      }
      publisher_->publish(*cv_bridge::CvImage(image->header,"bgr8",output).toImageMsg());
    } catch (const cv_bridge::Exception& error) { RCLCPP_WARN(get_logger(),"%s",error.what()); }
  }
  cv::Mat map1_,map2_,K_;
  message_filters::Subscriber<Cloud> points_,lines_;
  message_filters::Subscriber<Image> images_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> synchronizer_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<FeatureOverlay>()); }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("image_node_b"),"%s",error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
