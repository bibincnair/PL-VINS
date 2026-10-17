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
auto pub_lobs = n->create_publisher<sensor_msgs::msg::PointCloud>("/stereolinefeature_tracker/linefeature", 1000);

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

        stringstream ss;
        ss << sim_file+"keyframe/all_points_"<< i <<".txt";
        std::vector<Eigen::Vector2d> pobs;
        LoadPointObs(ss.str(),pobs);

        stringstream ss1,ss2;
        ss1 << sim_file+"keyframe/all_lines_"<< i <<".txt";
        ss2 << sim_file+"keyframe/all_lines_R_"<< i <<".txt";
        std::vector<Eigen::Vector4d> lobs;
        LoadLineObs(ss1.str(),lobs);
        std::vector<Eigen::Vector4d> lobs_R;
        LoadLineObs(ss2.str(),lobs_R);

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
            u_of_point.values.push_back((float)pobs[j].x());
            v_of_point.values.push_back((float)pobs[j].y());
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        //RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_points->header.stamp), rclcpp::Clock().now().seconds());
        pub_pobs->publish(*feature_points);

            // line feature
            sensor_msgs::msg::PointCloud::SharedPtr feature_lines(new sensor_msgs::msg::PointCloud);
            sensor_msgs::msg::ChannelFloat32 id_of_line;   //  feature id
            sensor_msgs::msg::ChannelFloat32 u_of_endpoint;    //  u
            sensor_msgs::msg::ChannelFloat32 v_of_endpoint;    //  v
            // right camera matched line
            sensor_msgs::msg::ChannelFloat32 u_of_startpoint_R;    //  u
            sensor_msgs::msg::ChannelFloat32 v_of_startpoint_R;    //  v
            sensor_msgs::msg::ChannelFloat32 u_of_endpoint_R;    //  u
            sensor_msgs::msg::ChannelFloat32 v_of_endpoint_R;    //  v

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

                u_of_startpoint_R.values.push_back(lobs_R[j](0));
                v_of_startpoint_R.values.push_back(lobs_R[j](1));
                u_of_endpoint_R.values.push_back(lobs_R[j](2));
                v_of_endpoint_R.values.push_back(lobs_R[j](3));

            }
            feature_lines->channels.push_back(id_of_line);
            feature_lines->channels.push_back(u_of_endpoint);
            feature_lines->channels.push_back(v_of_endpoint);

            feature_lines->channels.push_back(u_of_startpoint_R);
            feature_lines->channels.push_back(v_of_startpoint_R);
            feature_lines->channels.push_back(u_of_endpoint_R);
            feature_lines->channels.push_back(v_of_endpoint_R);
//        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_lines->header.stamp), rclcpp::Clock().now().seconds());
        pub_lobs->publish(*feature_lines);


        //done publishing
        rclcpp::spin_some(n);
        loop_rate.sleep();
    }
    return 0;
}
