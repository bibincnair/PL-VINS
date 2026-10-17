#include <plvins_common/ros.hpp>
#include <cstdio>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
template <typename T>
T readParam(const rclcpp::Node::SharedPtr &n, std::string name)
{
    T ans{};
    if (plvins::parameter(n, name, ans))
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("plvins"), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("plvins"), "Failed to load " << name);
        throw std::runtime_error("missing parameter: " + name);
    }
    return ans;
}

struct Data
{
    Data(FILE *f)
    {
        valid = fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) == 17;
        if (!valid) return;
        t /= 1e9;
    }
    bool valid=false;
    double t=0;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};
int idx = 1;
vector<Data> benchmark;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
nav_msgs::msg::Path path;

int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;


void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
{
    //RCLCPP_INFO(rclcpp::get_logger("plvins"), "odom callback!");
    if (plvins::seconds(odom_msg->header.stamp) > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= plvins::seconds(odom_msg->header.stamp); idx++)
        ;


    if (init++ < SKIP)
    {
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = plvins::stamp(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom->publish(odometry);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path->publish(path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("benchmark_publisher");

    string csv_file = readParam<string>(n, "data_name");
    std::cout << "load ground truth " << csv_file << std::endl;
    FILE *f = fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
      RCLCPP_WARN(rclcpp::get_logger("plvins"), "can't load ground truth; wrong path");
      //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return 1;
    }
    char tmp[10000];
    if (!fgets(tmp, 10000, f)) { fclose(f); throw std::runtime_error("Empty ground truth CSV"); }
    while (true) {
        Data row(f);
        if (!row.valid) {
            if (!feof(f)) { fclose(f); throw std::runtime_error("Malformed ground truth CSV row"); }
            break;
        }
        if (!std::isfinite(row.t) || (!benchmark.empty() && row.t<=benchmark.back().t)) {
            fclose(f); throw std::runtime_error("Ground truth timestamps must increase");
        }
        benchmark.push_back(row);
    }
    fclose(f);
    if (benchmark.size()<2) throw std::runtime_error("ground truth CSV needs at least two rows");
    RCLCPP_INFO(rclcpp::get_logger("plvins"), "Data loaded: %d", (int)benchmark.size());

    pub_odom = n->create_publisher<nav_msgs::msg::Odometry>("~/odometry", 1000);
    pub_path = n->create_publisher<nav_msgs::msg::Path>("~/path", 1000);

    auto sub_odom = n->create_subscription<nav_msgs::msg::Odometry>("estimated_odometry", rclcpp::QoS(1000), odom_callback);
    
    rclcpp::Rate r(20);
    rclcpp::spin(n);
}
