#include "dataset_io.h"
#include <plvins_common/ros.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>


using namespace std;

using plvins::simulation::LoadPose;
using plvins::simulation::LoadPointObs;
using plvins::simulation::LoadLineObs;

int main(int argc, char **argv)
{
 
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("feature");
auto pub_pobs = n->create_publisher<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", 1000);

    rclcpp::Rate loop_rate(30);
    std::string sim_file;
    plvins::parameter(n,"sim_file_path",sim_file);
    if (sim_file.empty() || !std::filesystem::is_directory(sim_file)) throw std::runtime_error("sim_file_path must name an external simulation directory");
    sim_file=std::filesystem::absolute(sim_file).string()+"/";

    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;
    LoadPose(sim_file+"cam_pose.txt",timestamp, gyros, accs);

    sleep(1);   //等vins代码启动
    //while (rclcpp::ok())
    for (int i = 0; rclcpp::ok() && i < timestamp.size(); ++i)
    {

        stringstream ss, ss_r;
        ss << sim_file+"keyframe/all_points_"<< i <<".txt";
        ss_r << sim_file+"keyframe/all_points_r_"<< i <<".txt";
        std::vector<Eigen::Vector2d> pobs,pobs_r;
        LoadPointObs(ss.str(),pobs);
        LoadPointObs(ss_r.str(),pobs_r);

        // point feature
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;   //  feature id
        sensor_msgs::msg::ChannelFloat32 u_of_point;    //  u
        sensor_msgs::msg::ChannelFloat32 v_of_point;    //  v

        auto t = plvins::stamp(timestamp[i]);
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
            u_of_point.values.push_back((float)pobs_r[j].x());
            v_of_point.values.push_back((float)pobs_r[j].y());
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        //RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_points->header.stamp), rclcpp::Clock().now().seconds());
        pub_pobs->publish(*feature_points);

        //done publishing
        rclcpp::spin_some(n);
        loop_rate.sleep();
    }
    return 0;
}
