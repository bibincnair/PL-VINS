#include "dataset_io.h"
#include <thread>
#include <chrono>
#include <plvins_common/ros.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
using namespace std;

using plvins::simulation::LoadPose;
using plvins::simulation::LoadPointObs;
using plvins::simulation::LoadLineObs;

int main(int argc, char **argv)
{
 
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("imu_features");
auto out_pub = n->create_publisher<sensor_msgs::msg::Imu >("/imu0", 1000);
auto pub_pobs = n->create_publisher<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", 1000);
auto pub_lobs = n->create_publisher<sensor_msgs::msg::PointCloud>("/linefeature_tracker/linefeature", 1000);

    const double playback_rate=n->declare_parameter<double>("playback_rate",1.);
    if (!std::isfinite(playback_rate) || playback_rate<=0.) throw std::runtime_error("playback_rate must be positive");
    std::string sim_file;
    plvins::parameter(n,"sim_file_path",sim_file);
    if (sim_file.empty() || !std::filesystem::is_directory(sim_file)) throw std::runtime_error("sim_file_path must name an external simulation directory");
    sim_file=std::filesystem::absolute(sim_file).string()+"/";


    std::vector<double> imutimestamp, camtimestamp;
    std::vector<Eigen::Vector3d> gyros, temp1;
    std::vector<Eigen::Vector3d> accs, temp2;
    LoadPose(sim_file+"imu_pose.txt",imutimestamp, gyros, accs);
    LoadPose(sim_file+"cam_pose.txt",camtimestamp, temp1, temp2);

    //while (rclcpp::ok())
    if (imutimestamp.empty() || camtimestamp.empty()) throw std::runtime_error("Simulation input is empty");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto started=std::chrono::steady_clock::now();
    int cam_time_index = 0;
    for (int i = 0; rclcpp::ok() && i < imutimestamp.size(); ++i)
    {
        std::this_thread::sleep_until(started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>((imutimestamp[i]-imutimestamp.front())/playback_rate)));
        //new imu message
        sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();

        auto t = plvins::stamp(imutimestamp[i]);
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

        while (cam_time_index < static_cast<int>(camtimestamp.size()) && camtimestamp[cam_time_index]<=imutimestamp[i])
        {
            stringstream ss;
            ss << sim_file+"keyframe/all_points_"<< cam_time_index <<".txt";
            std::vector<Eigen::Vector2d> pobs;
            LoadPointObs(ss.str(),pobs);

            stringstream ss1;
            ss1 << sim_file+"keyframe/all_lines_"<< cam_time_index <<".txt";
            std::vector<Eigen::Vector4d> lobs;
            LoadLineObs(ss1.str(),lobs);

            // point feature
            sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
            sensor_msgs::msg::ChannelFloat32 id_of_point;   //  feature id
            sensor_msgs::msg::ChannelFloat32 u_of_point;    //  u
            sensor_msgs::msg::ChannelFloat32 v_of_point;    //  v

            auto t = plvins::stamp(camtimestamp[cam_time_index]);
            feature_points->header.stamp = t;
            feature_points->header.frame_id = "world";

            for (int j = 0; j < pobs.size(); ++j) {
                int p_id = j;
                geometry_msgs::msg::Point32 p;
                p.x = pobs[j].x();
                p.y = pobs[j].y();
                p.z = 1;

                feature_points->points.push_back(p);
                id_of_point.values.push_back(p_id * 1.0);
                u_of_point.values.push_back((float)pobs[j].x());
                v_of_point.values.push_back((float)pobs[j].y());
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            RCLCPP_INFO(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_points->header.stamp), rclcpp::Clock().now().seconds());
            pub_pobs->publish(*feature_points);

            // line feature
            sensor_msgs::msg::PointCloud::SharedPtr feature_lines(new sensor_msgs::msg::PointCloud);
            sensor_msgs::msg::ChannelFloat32 id_of_line;   //  feature id
            sensor_msgs::msg::ChannelFloat32 u_of_endpoint;    //  u
            sensor_msgs::msg::ChannelFloat32 v_of_endpoint;    //  v

            feature_lines->header = feature_points->header;
            feature_lines->header.frame_id = "world";

            for (unsigned int j = 0; j < lobs.size(); j++)
            {

                int p_id = j;
                geometry_msgs::msg::Point32 p;
                p.x = lobs[j](0);
                p.y = lobs[j](1);
                p.z = 1;

                feature_lines->points.push_back(p);
                id_of_line.values.push_back((float)p_id);
                u_of_endpoint.values.push_back(lobs[j](2));
                v_of_endpoint.values.push_back(lobs[j](3));
                //PLVINS_ASSERT(inBorder(cur_pts[j]));
            }
            feature_lines->channels.push_back(id_of_line);
            feature_lines->channels.push_back(u_of_endpoint);
            feature_lines->channels.push_back(v_of_endpoint);
//        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_lines->header.stamp), rclcpp::Clock().now().seconds());
            pub_lobs->publish(*feature_lines);

            cam_time_index++;

        }




        //done publishing
        rclcpp::spin_some(n);

    }
    return 0;
}
