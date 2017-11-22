#pragma once
#include <vector>
#include <opencv2/core/core.hpp>

class Ball{

public:
    /*
     *  each ball has a name , radius and center
     *  name is for tracking purpose since we have three similar balls
     */

    Ball(std::string name,double radius,cv::Point2f center);
    std::string getName();
    double getRadius();
    cv::Point2f getCenter();
    void setName(std::string name);
    std::vector<Ball> findBalls(cv::Mat frame , cv::Mat Mask);
    void markBalls(std::vector<Ball> balls , cv::Mat img);

private:
    std::string Name;
    double Radius;
    cv::Point2f Center;
};
