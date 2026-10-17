#include <plvins_common/ros.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>


#include "linefeature_tracker.h"

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

class LineTrackerNode final : public rclcpp::Node {
 public:
  LineTrackerNode(): Node("linefeature_tracker") {}
  void start() {

    auto n = shared_from_this();
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);

    RCLCPP_INFO(rclcpp::get_logger("plvins"), "start line feature");
    image_sub_ = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::SensorDataQoS().keep_last(100), [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        try { img_callback(message); }
        catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "Image rejected: %s", error.what()); }
    });

    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("~/linefeature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("~/linefeature_img",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

  }
 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  int64_t last_stamp=-1, last_bucket=-1;
vector<uchar> r_status;
vector<float> r_err;


rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;

LineFeatureTracker trackerData;

int pub_count = 1;

double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;
void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    if (img_msg->height < static_cast<unsigned>(ROW) || img_msg->width != static_cast<unsigned>(COL)) {
        RCLCPP_WARN(rclcpp::get_logger("plvins"), "Image dimensions do not match calibration"); return;
    }
    TicToc t_r;

    const int64_t stamp_ns=plvins::nanoseconds(img_msg->header.stamp);
    if (stamp_ns<=last_stamp) return;
    last_stamp=stamp_ns;
    const int64_t bucket = FREQ>0 ? (stamp_ns/1000000000LL)*FREQ + (stamp_ns%1000000000LL)*FREQ/1000000000LL : stamp_ns;
    PUB_THIS_FRAME = bucket!=last_bucket;
    if (PUB_THIS_FRAME) last_bucket=bucket;
    if (PUB_THIS_FRAME)
    {
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
//    cv::imshow("lineimg",show_img);
//    cv::waitKey(1);
    
    frame_cnt++;
    trackerData.readImage(ptr->image.rowRange(0 , ROW));   // rowRange(i,j) 取图像的i～j行

        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_lines(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::msg::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::msg::ChannelFloat32 v_of_endpoint;    //  v

        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                auto un_lines = trackerData.undistortedLineEndPoints();

                //auto &cur_lines = trackerData.curframe_->vecLine;
                auto &ids = trackerData.curframe_->lineID;

                for (unsigned int j = 0; j < ids.size(); j++)
                {

                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_lines[j].StartPt.x;
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);
                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    //PLVINS_ASSERT(inBorder(cur_pts[j]));
                }
            }

        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);
        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_lines->header.stamp), rclcpp::Clock().now().seconds());
        pub_img->publish(*feature_lines);

    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
    // RCLCPP_INFO(rclcpp::get_logger("plvins"), "whole Line feature tracker processing costs: %f", mean_time);
}

};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<LineTrackerNode>(); node->start(); rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("linefeature_tracker"), "%s", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
