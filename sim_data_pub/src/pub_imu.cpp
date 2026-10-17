#include "dataset_io.h"
#include <plvins_common/ros.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
using namespace std;

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


using plvins::simulation::LoadPose;
using plvins::simulation::LoadPointObs;
using plvins::simulation::LoadLineObs;

int main(int argc, char **argv)
{
 
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("imu");
    
auto out_pub = n->create_publisher<sensor_msgs::msg::Imu >("/imu0", 1000);
    rclcpp::Rate loop_rate(200);
    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;

    std::string sim_file;
    plvins::parameter(n,"sim_file_path",sim_file);
    if (sim_file.empty() || !std::filesystem::is_directory(sim_file)) throw std::runtime_error("sim_file_path must name an external simulation directory");
    sim_file=std::filesystem::absolute(sim_file).string()+"/";

    LoadPose(sim_file + "imu_pose.txt",timestamp, gyros, accs);

    sleep(1);
    //while (rclcpp::ok())
    for (int i = 0; rclcpp::ok() && i < timestamp.size(); ++i)
    {
        //new imu message
        sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();

        auto t = plvins::stamp(timestamp[i]);
        imu_msg.header.stamp = t;
        imu_msg.header.frame_id = "imu0";

        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1.0;
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        Eigen::Vector3d gyro = gyros[i];
        Eigen::Vector3d acc = accs[i];
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        //publish the new imu message
        out_pub->publish(imu_msg);
        RCLCPP_INFO(rclcpp::get_logger("plvins"), "%s", "send an imu message");

        //done publishing
        rclcpp::spin_some(n);
        loop_rate.sleep();
    }
    return 0;
}
