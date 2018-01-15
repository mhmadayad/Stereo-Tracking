#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <utility>
class Ball{

public:
    /*
     *  each ball has a name , radius and center
     *  name is for tracking purpose since we have three similar balls
     */

    Ball(std::string name,double radius,cv::Point2f center, std::vector<cv::Point> contours);
    std::string getName();
    std::vector<cv::Point> getContour();
    double getRadius();
    cv::Point2f getCenter();
    void setName(std::string name);
    std::pair<std::vector<Ball>,cv::Mat> findBalls(cv::Mat frame , int scenario);
    //tracking(a,b,c) based on distance between the centers
    void markBalls(std::vector<Ball> &balls , cv::Mat &img, int scenario);


private:
    std::string name;
    double radius;
    cv::Point2f center;
    std::vector<cv::Point> contour;
    int scenario=1;//1 i.e. 3 balls , 2 i.e. 2 balls
};
