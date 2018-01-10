#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/calib3d.hpp"
#include <utility>

using namespace cv;
using namespace std;
class DepthMap{

public:
    DepthMap(int x);
    Mat stereoBM(Mat leftimg, Mat rightimg ,Ptr<StereoBM> sbm);
    Mat stereoSGBM(Mat leftimg, Mat rightimg, Ptr<StereoSGBM> sgbm);
    Mat triangulatePts(vector<Point2d> leftPoints, vector<Point2d> rightPoints,Mat proj1,Mat proj2);

private:

};
