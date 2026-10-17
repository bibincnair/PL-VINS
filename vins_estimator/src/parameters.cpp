#include <plvins_common/ros.hpp>
#include "parameters.h"

double BASE_LINE;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES;
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

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

void readParameters(const rclcpp::Node::SharedPtr &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        throw std::runtime_error("Cannot open config_file: " + config_file);
    }

    VINS_FOLDER_PATH = plvins::output_directory(n);
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;

    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    VINS_RESULT_PATH = VINS_FOLDER_PATH;
    EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + "/extrinsic_calibration.yaml";
    std::ofstream(VINS_RESULT_PATH + "/trajectory_ns.txt", std::ios::trunc);
    std::ofstream(VINS_RESULT_PATH + "/trajectory_tum.txt", std::ios::trunc);

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("plvins"), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            RCLCPP_WARN(rclcpp::get_logger("plvins"), " Optimize extrinsic param around initial guess!");
        }
        if (ESTIMATE_EXTRINSIC == 0)
            RCLCPP_WARN(rclcpp::get_logger("plvins"), " fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("plvins"), "Extrinsic_R : " << std::endl << RIC[0]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("plvins"), "Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    LOOP_CLOSURE = fsSettings["loop_closure"];
    CAM_NAMES = config_file;

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM = 1000;

    if (!fsSettings["model_type"].isNone())
        BASE_LINE = static_cast<double>(fsSettings["base_line"]);
    else
        BASE_LINE = - 1.0;

    fsSettings.release();
}
