#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <utility>
class LED{

public:
    /*
     *  each led has a name , radius and center
     *  name is for tracking purpose since we have three similar leds
     */

    LED(std::string name,double radius,cv::Point2f center, std::vector<cv::Point> contours);
    std::string getName();
    std::vector<cv::Point> getContour();
    double getRadius();
    cv::Point2f getCenter();
    void setName(std::string name);
    std::vector<LED> findLEDs(cv::Mat frame , int scenario,int num);
    //tracking(a,b,c) based on distance between the centers
    void markLEDs(std::vector<LED> &LEDs , cv::Mat &img, int scenario);


private:
    std::string name;
    double radius;
    cv::Point2f center;
    std::vector<cv::Point> contour;
    int scenario=1;//1 i.e. 3 leds , 2 i.e. 2 leds
};
