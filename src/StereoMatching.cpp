#include "StereoMatching.h"
#include "Constants.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <utility>
#include <a.out.h>
#include <cmath>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/core/utility.hpp>

DepthMap::DepthMap(int x ){x=1;}

Mat DepthMap::stereoBM(cv::Mat leftimg , cv::Mat rightimg, cv::Ptr<cv::StereoBM> sbm){
    Mat imgDisparity16S= Mat( leftimg.rows, rightimg.cols, CV_16U );
    sbm->setBlockSize(5);
    sbm->setMinDisparity(0);
    sbm->setTextureThreshold(500);
    sbm->setUniquenessRatio(0);
    sbm->setPreFilterSize(5);
    sbm->setPreFilterCap(61);
    sbm->setSpeckleWindowSize(10);
    sbm->setSpeckleRange(80);
    sbm->setDisp12MaxDiff(20);
    sbm->compute( leftimg, rightimg, imgDisparity16S );

    return imgDisparity16S;
}

Mat DepthMap::stereoSGBM(Mat leftimg, Mat rightimg, Ptr<StereoSGBM> sgbm){
    Mat imgDisparity16S= Mat( leftimg.rows, rightimg.cols, CV_16U );
    sgbm->setPreFilterCap(63);
    sgbm->setBlockSize(5);
    sgbm->setP1(100);
    sgbm->setP2(100);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(16);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    //sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    //sgbm->setMode(StereoSGBM::MODE_HH);
    sgbm->compute(leftimg, rightimg, imgDisparity16S);

    return imgDisparity16S;
}

Mat DepthMap::triangulatePts(vector<Point2d> leftPoints, vector<Point2d> rightPoints , Mat proj1, Mat proj2){
    ///TODO::undistort pts

    Mat pnts3D(4 /*4*/,leftPoints.size(),CV_64F);
    triangulatePoints(proj1,proj2,leftPoints,rightPoints,pnts3D);
    return pnts3D;
}
