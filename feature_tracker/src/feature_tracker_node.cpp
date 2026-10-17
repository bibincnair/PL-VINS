#include <plvins_common/ros.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>


#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

class PointTrackerNode final : public rclcpp::Node {
 public:
  PointTrackerNode(): Node("feature_tracker") {}
  void start() {

    auto n = shared_from_this();
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCLCPP_INFO(rclcpp::get_logger("plvins"), "load mask fail");
                throw std::runtime_error("invalid runtime state");
            }
            else
                RCLCPP_INFO(rclcpp::get_logger("plvins"), "load mask success");
        }
    }

    image_sub_ = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::SensorDataQoS().keep_last(100), [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        try { img_callback(message); }
        catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "Image rejected: %s", error.what()); }
    });

    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("~/feature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("~/feature_img",1000);
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

FeatureTracker trackerData[NUM_OF_CAM];

int pub_count = 1;


double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;
void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    if (img_msg->height < static_cast<unsigned>(ROW) || img_msg->width != static_cast<unsigned>(COL)) {
        RCLCPP_WARN(rclcpp::get_logger("plvins"), "Image dimensions do not match calibration"); return;
    }

    const int64_t stamp_ns=plvins::nanoseconds(img_msg->header.stamp);
    if (stamp_ns<=last_stamp) return;
    last_stamp=stamp_ns;
    const int64_t bucket = FREQ>0 ? (stamp_ns/1000000000LL)*FREQ + (stamp_ns%1000000000LL)*FREQ/1000000000LL : stamp_ns;
    PUB_THIS_FRAME = bucket!=last_bucket;
    if (PUB_THIS_FRAME) last_bucket=bucket;
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;

    TicToc t_r;
    frame_cnt++;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)     // 单目 或者 非双目,   #### readImage： 这个函数里 做了很多很多事, 提取特征 ####
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)));   // rowRange(i,j) 取图像的i～j行
        else
        {
            if (EQUALIZE)      // 对图像进行直方图均衡化处理
            {
                std::cout <<" EQUALIZE " << std::endl;
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
//        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        trackerData[i].showUndistortion();
#endif
    }

    // 双目
    if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
    {
        pub_count++;
        r_status.clear();
        r_err.clear();
        TicToc t_o;
        cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "spatial optical flow costs: %fms", t_o.toc());
        vector<cv::Point2f> ll, rr;
        vector<int> idx;
        for (unsigned int i = 0; i < r_status.size(); i++)
        {
            if (!inBorder(trackerData[1].cur_pts[i]))
                r_status[i] = 0;

            if (r_status[i])
            {
                idx.push_back(i);

                Eigen::Vector3d tmp_p;
                trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

                trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            }
        }
        if (ll.size() >= 8)
        {
            vector<uchar> status;
            TicToc t_f;
            cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
            RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "find f cost: %f", t_f.toc());
            int r_cnt = 0;
            for (unsigned int i = 0; i < status.size(); i++)
            {
                if (status[i] == 0)
                    r_status[idx[i]] = 0;
                r_cnt += r_status[idx[i]];
            }
        }
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)                   // 单目 或者 非双目
                completed |= trackerData[j].updateID(i);   // 更新检测到的特征的id, 如果一个特征是新的，那就赋予新的id，如果一个特征已经有了id，那就不处理
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;   //  feature id
        sensor_msgs::msg::ChannelFloat32 u_of_point;    //  u
        sensor_msgs::msg::ChannelFloat32 v_of_point;    //  v

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                auto un_pts = trackerData[i].undistortedPoints();
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    PLVINS_ASSERT(inBorder(cur_pts[j]));
                }
            }
            else if (STEREO_TRACK)
            {
                auto r_un_pts = trackerData[1].undistortedPoints();
                auto &ids = trackerData[0].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (r_status[j])
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::msg::Point32 p;
                        p.x = r_un_pts[j].x;
                        p.y = r_un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    }
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        RCLCPP_DEBUG(rclcpp::get_logger("plvins"), "publish %f, at %f", plvins::seconds(feature_points->header.stamp), rclcpp::Clock().now().seconds());
        pub_img->publish(*feature_points);

//        if (SHOW_TRACK)  // SHOW_TRACK
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);

            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);
//                tmp_img = trackerData[0].cur_img;
                cv::cvtColor(trackerData[0].cur_img, tmp_img, cv::COLOR_GRAY2RGB);
                if (i != 1 || !STEREO_TRACK)
                {
                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                        //char name[10];
                        //sprintf(name, "%d", trackerData[i].ids[j]);
                        //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                    }
                }
                else
                {
                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        if (r_status[j])
                        {
                            cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255, 0), 2);
                            cv::line(stereo_img, trackerData[i - 1].cur_pts[j], trackerData[i].cur_pts[j] + cv::Point2f(0, ROW), cv::Scalar(0, 255, 0));
                        }
                    }
                }
            }

            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);

            pub_match->publish(*ptr->toImageMsg());
        }
    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
//    RCLCPP_INFO(rclcpp::get_logger("plvins"), "whole point feature tracker processing costs: %f", t_r.toc());
    RCLCPP_INFO(rclcpp::get_logger("plvins"), "whole point feature tracker processing costs: %f", mean_time);

}

};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<PointTrackerNode>(); node->start(); rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("feature_tracker"), "%s", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
