#include "solve_5pts.h"

bool MotionEstimator::solveRelativeRT(const vector<pair<Vector3d,Vector3d>>& correspondences,
                                     Matrix3d& rotation,Vector3d& translation) {
    if (correspondences.size()<15) return false;
    vector<cv::Point2f> first,second;
    for (const auto& observation:correspondences) {
        if (!observation.first.allFinite() || !observation.second.allFinite()) return false;
        first.emplace_back(observation.first.x(),observation.first.y());
        second.emplace_back(observation.second.x(),observation.second.y());
    }
    cv::Mat mask;
    cv::Mat essential=cv::findFundamentalMat(first,second,cv::FM_RANSAC,.3/460.,.99,mask);
    // Degenerate correspondences can produce an empty model. Never reshape or decompose it.
    if (essential.rows!=3 || essential.cols!=3 || !cv::checkRange(essential)) return false;
    essential=essential.clone();
    const cv::Mat camera=cv::Mat::eye(3,3,CV_64F);
    cv::Mat R,t;
    // OpenCV 5 provides this implementation; do not override symbols in namespace cv.
    const int inliers=cv::recoverPose(essential,first,second,camera,R,t,mask);
    if (inliers<=12) return false;
    Matrix3d relative_rotation; Vector3d relative_translation;
    for (int row=0;row<3;++row) {
        relative_translation[row]=t.at<double>(row,0);
        for (int col=0;col<3;++col) relative_rotation(row,col)=R.at<double>(row,col);
    }
    rotation=relative_rotation.transpose();
    translation=-rotation*relative_translation;
    return rotation.allFinite() && translation.allFinite();
}
