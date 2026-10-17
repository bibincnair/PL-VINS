#include <plvins_common/ros.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

namespace {
using Imu = sensor_msgs::msg::Imu;
using Features = sensor_msgs::msg::PointCloud;
constexpr size_t kImuLimit = 4000, kFrameLimit = 100;

bool validFeatures(const Features& msg, size_t channels) {
  if (msg.channels.size() < channels) return msg.points.empty();
  for (size_t c=0; c<channels; ++c) {
    if (msg.channels[c].values.size() != msg.points.size()) return false;
    for (float v: msg.channels[c].values) if (!std::isfinite(v)) return false;
  }
  for (const auto& p: msg.points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  return true;
}

class EstimatorNode final : public rclcpp::Node {
 public:
  EstimatorNode(): Node("plvins_estimator") {}
  void start() {
    auto node = shared_from_this();
    readParameters(node);
    estimator_.setParameter();
    registerPub(node);
    imu_sub_ = create_subscription<Imu>(IMU_TOPIC, rclcpp::SensorDataQoS().keep_last(2000),
      [this](Imu::ConstSharedPtr msg) { onImu(msg); });
    point_sub_ = create_subscription<Features>("/feature_tracker/feature", 100,
      [this](Features::ConstSharedPtr msg) { onFeatures(msg, false); });
    line_sub_ = create_subscription<Features>("/linefeature_tracker/linefeature", 100,
      [this](Features::ConstSharedPtr msg) { onFeatures(msg, true); });
    relo_sub_ = create_subscription<Features>("/pose_graph/match_points", 10,
      [this](Features::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(buffer_mutex_); relocation_=msg; });
    worker_ = std::thread([this] { process(); });
  }
  ~EstimatorNode() override { stop(); }
  bool failed() const { return failed_.load(); }
  void stop() {
    { std::lock_guard<std::mutex> lock(buffer_mutex_); stopping_=true; }
    ready_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
 private:
  struct Batch { Features::ConstSharedPtr points, lines, relocation; std::vector<Imu::ConstSharedPtr> imus; };
  void onFeatures(Features::ConstSharedPtr msg, bool line) {
    if (!validFeatures(*msg, 3)) { RCLCPP_WARN(get_logger(), "Discarding malformed feature frame"); return; }
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto& queue = line ? lines_ : points_;
    auto& last = line ? last_line_ : last_point_;
    const auto t = plvins::nanoseconds(msg->header.stamp);
    if (t <= last) return;
    last=t; queue.push_back(msg);
    if (queue.size()>kFrameLimit) queue.pop_front();
    ready_.notify_one();
  }
  static Eigen::Vector3d acceleration(const Imu& msg) {
    return {msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z};
  }
  static Eigen::Vector3d angularVelocity(const Imu& msg) {
    return {msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z};
  }
  void onImu(Imu::ConstSharedPtr msg) {
    if (!acceleration(*msg).allFinite() || !angularVelocity(*msg).allFinite()) return;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      const auto t=plvins::nanoseconds(msg->header.stamp);
      if (t<=last_imu_) return;
      last_imu_=t; imus_.push_back(msg);
      if (imus_.size()>kImuLimit) imus_.pop_front();
    }
    ready_.notify_one();
    std::lock_guard<std::mutex> lock(prediction_mutex_);
    if (prediction_ready_) {
      predict(*msg);
      auto header=msg->header; header.frame_id="world";
      pubLatestOdometry(pred_p_,pred_q_,pred_v_,header);
    }
  }
  bool batch(Batch& result) {
    while (!points_.empty() && !lines_.empty()) {
      const auto p=plvins::nanoseconds(points_.front()->header.stamp);
      const auto l=plvins::nanoseconds(lines_.front()->header.stamp);
      if (p<l) { points_.pop_front(); continue; }
      if (l<p) { lines_.pop_front(); continue; }
      if (imus_.empty() || plvins::nanoseconds(imus_.back()->header.stamp)<=p) return false;
      if (current_time_<0 && plvins::nanoseconds(imus_.front()->header.stamp)>=p) {
        points_.pop_front(); lines_.pop_front(); continue;
      }
      result.points=points_.front(); points_.pop_front();
      result.lines=lines_.front(); lines_.pop_front();
      while (!imus_.empty() && plvins::nanoseconds(imus_.front()->header.stamp)<=p) {
        result.imus.push_back(imus_.front()); imus_.pop_front();
      }
      result.imus.push_back(imus_.front()); // retain the sample after the image for interpolation
      result.relocation=relocation_; relocation_.reset();
      return true;
    }
    return false;
  }
  void integrate(const Batch& data) {
    const double image_time=plvins::seconds(data.points->header.stamp);
    for (const auto& msg:data.imus) {
      const double t=plvins::seconds(msg->header.stamp);
      Eigen::Vector3d acc=acceleration(*msg), gyro=angularVelocity(*msg);
      if (current_time_<0) { current_time_=t; previous_acc_=acc; previous_gyro_=gyro; estimator_.processIMU(0.,acc,gyro); }
      if (t<=current_time_) continue;
      const double target=std::min(t,image_time);
      if (target<=current_time_) continue;
      const double fraction=(target-current_time_)/(t-current_time_);
      acc=previous_acc_+fraction*(acc-previous_acc_);
      gyro=previous_gyro_+fraction*(gyro-previous_gyro_);
      estimator_.processIMU(target-current_time_,acc,gyro);
      current_time_=target; previous_acc_=acc; previous_gyro_=gyro;
    }
  }
  void predict(const Imu& msg) {
    const double t=plvins::seconds(msg.header.stamp), dt=t-pred_time_;
    if (dt<=0) return;
    const auto acc=acceleration(msg), gyro=angularVelocity(msg);
    const Eigen::Vector3d a0=pred_q_*(pred_acc_-pred_ba_)-pred_g_;
    pred_q_=(pred_q_*Utility::deltaQ((.5*(pred_gyro_+gyro)-pred_bg_)*dt)).normalized();
    const Eigen::Vector3d a1=pred_q_*(acc-pred_ba_)-pred_g_;
    const Eigen::Vector3d a=.5*(a0+a1);
    pred_p_+=dt*pred_v_+.5*dt*dt*a; pred_v_+=dt*a;
    pred_acc_=acc; pred_gyro_=gyro; pred_time_=t;
  }
  void process() {
    try {
      while (true) {
        Batch data;
        {
          std::unique_lock<std::mutex> lock(buffer_mutex_);
          ready_.wait(lock,[&] { return stopping_ || batch(data); });
          if (stopping_) return;
        }
        integrate(data);
        if (data.relocation && !data.relocation->channels.empty() && data.relocation->channels[0].values.size()>=8) {
          const auto& v=data.relocation->channels[0].values;
          std::vector<Eigen::Vector3d> matches;
          for (const auto& p:data.relocation->points) matches.emplace_back(p.x,p.y,p.z);
          estimator_.setReloFrame(plvins::seconds(data.relocation->header.stamp),int(v[7]),matches,
            Eigen::Vector3d(v[0],v[1],v[2]),Eigen::Quaterniond(v[3],v[4],v[5],v[6]).normalized().toRotationMatrix());
        }
        std::map<int,std::vector<std::pair<int,Eigen::Matrix<double,5,1>>>> points;
        std::map<int,std::vector<std::pair<int,Eigen::Vector4d>>> lines;
        for (size_t i=0;i<data.points->points.size();++i) {
          const auto& p=data.points->points[i]; const int id=int(data.points->channels[0].values[i]+.5f);
          if (std::abs(p.z-1.f)>1e-6f || id<0) continue;
          Eigen::Matrix<double,5,1> observation;
          observation<<p.x,p.y,p.z,data.points->channels[1].values[i],data.points->channels[2].values[i];
          points[id/NUM_OF_CAM].emplace_back(id%NUM_OF_CAM,observation);
        }
        for (size_t i=0;i<data.lines->points.size();++i) {
          const auto& p=data.lines->points[i]; const int id=int(data.lines->channels[0].values[i]+.5f);
          if (id<0) continue;
          lines[id/NUM_OF_CAM].emplace_back(id%NUM_OF_CAM,Eigen::Vector4d(p.x,p.y,data.lines->channels[1].values[i],data.lines->channels[2].values[i]));
        }
        TicToc timer;
        estimator_.processImage(points,lines,data.points->header);
        printStatistics(estimator_,timer.toc());
        auto header=data.points->header; header.frame_id="world";
        const Eigen::Vector3d shift=Eigen::Vector3d::Zero(); const Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
        pubOdometry(estimator_,header,shift,rotation); pubKeyPoses(estimator_,header,shift,rotation);
        pubCameraPose(estimator_,header,shift,rotation); pubLinesCloud(estimator_,header,shift,rotation);
        pubPointCloud(estimator_,header,shift,rotation); pubTF(estimator_,header,shift,rotation); pubKeyframe(estimator_);
        if (data.relocation) pubRelocalization(estimator_);
        {
          std::scoped_lock lock(buffer_mutex_,prediction_mutex_);
          prediction_ready_=estimator_.solver_flag==Estimator::NON_LINEAR;
          pred_g_=estimator_.g;
          pred_time_=current_time_; pred_p_=estimator_.Ps[WINDOW_SIZE]; pred_q_=estimator_.Rs[WINDOW_SIZE];
          pred_v_=estimator_.Vs[WINDOW_SIZE]; pred_ba_=estimator_.Bas[WINDOW_SIZE]; pred_bg_=estimator_.Bgs[WINDOW_SIZE];
          pred_acc_=previous_acc_; pred_gyro_=previous_gyro_;
          if (prediction_ready_) for (const auto& msg:imus_) predict(*msg);
        }
      }
    } catch (const std::exception& error) {
      failed_=true;
      RCLCPP_FATAL(get_logger(),"Estimator worker failed: %s",error.what());
      rclcpp::shutdown();
    }
  }
  std::atomic<bool> failed_{false};
  Estimator estimator_;
  std::mutex buffer_mutex_,prediction_mutex_;
  std::condition_variable ready_; std::thread worker_; bool stopping_=false,prediction_ready_=false;
  std::deque<Imu::ConstSharedPtr> imus_;
  std::deque<Features::ConstSharedPtr> points_,lines_; Features::ConstSharedPtr relocation_;
  int64_t last_imu_=-1,last_point_=-1,last_line_=-1;
  double current_time_=-1,pred_time_=0;
  Eigen::Vector3d previous_acc_=Eigen::Vector3d::Zero(),previous_gyro_=Eigen::Vector3d::Zero();
  Eigen::Vector3d pred_g_,pred_p_,pred_v_,pred_ba_,pred_bg_,pred_acc_,pred_gyro_;
  Eigen::Quaterniond pred_q_=Eigen::Quaterniond::Identity();
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Features>::SharedPtr point_sub_,line_sub_,relo_sub_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<EstimatorNode>(); node->start();
    rclcpp::spin(node); node->stop(); const bool failed=node->failed(); node.reset(); rclcpp::shutdown();
    if (failed) return 1;
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("plvins_estimator"),"%s",error.what());
    rclcpp::shutdown(); return 1;
  }
  return 0;
}
