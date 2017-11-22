#include "Ball.h"
#include "Constants.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <a.out.h>
#include <cmath>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/core/utility.hpp>

Ball::Ball(std::string name, double radius, cv::Point2f center)
      : Name(name),Radius(radius), Center(center){
}

double Ball::getRadius(){
    return Radius;
}

std::string Ball::getName(){
    return Name;
}

cv::Point2f Ball::getCenter(){
    return Center;
}

void Ball::setName(std::string name){
    Name=name;
}

//Balls detection
std::vector<Ball> Ball::findBalls(cv::Mat frame, cv::Mat Mask){
    std::vector<Ball> balls;
    cv::Mat grayImage , threshold;
    cv::cvtColor( frame, grayImage, CV_BGR2GRAY );
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    // Detect edges using Threshold
    cv::threshold( grayImage, threshold, 100, 255, cv::THRESH_BINARY );
    // Find contours
    cv::findContours( threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    //std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f> center( contours.size() );
    //std::vector<cv::Point2f> centers;
    std::vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ){

        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
    }

    /// Draw polygonal contour + bonding rects + circles
    cv::Mat drawing = cv::Mat::zeros( threshold.size(), CV_8UC3 );
    cv::Mat mask = cv::Mat::zeros( threshold.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
        //find balls between min and max radius
        if((int)radius[i] >minRadius && (int)radius[i]<maxRadius){
            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );

            //balls bounding box
            rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );

            //draw circles around the balls
            cv::circle( mask, center[i], (int)radius[i], CV_RGB(0,255,0), CV_FILLED, 8, 0 );
            cv::circle( drawing, center[i], (int)radius[i], CV_RGB(0,255,0), 2, 8, 0 );
            balls.push_back(Ball("ball",radius[i],center[i]));
        }
    }

    if(balls.size()==3){
    //draw the triangle
        markBalls(balls,drawing);
        //triange from the centers of three balls
        cv::line(drawing, balls[0].getCenter(), balls[1].getCenter(), CV_RGB(255,0,0),2);
        cv::line(drawing, balls[1].getCenter(), balls[2].getCenter(), CV_RGB(255,0,0),2);
        cv::line(drawing, balls[2].getCenter(), balls[0].getCenter(), CV_RGB(255,0,0),2);
        //markCircles(centers,drawing);

        //center of triangle
        int posX=( balls[0].getCenter().x + balls[1].getCenter().x +  balls[2].getCenter().x )/3;
        int posY=(  balls[0].getCenter().y +  balls[1].getCenter().y + balls[2].getCenter().y )/3;
        cv::putText(drawing, "center", cv::Point(posX,posY), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);

    }
    else if(balls.size()>3)
        std::cout << "noise-> detected more than 3 balls" <<std::endl;
    else
        std::cout <<balls.size() << " detected" << std::endl;

    // Show in a window
    cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cv::imshow( "Contours", drawing );
    cv::cvtColor( mask, mask, CV_BGR2GRAY );
    //cv::imshow( "mask", mask );
    mask.copyTo(Mask);
    return balls;
}


//Mark balls (A,B,C) despite the transformation takes place in next frame
//tracking
void Ball::markBalls(std::vector<Ball> balls , cv::Mat img){
    //distances between the balls. i.e-> triangle sides
    double d1 = cv::norm(balls[0].getCenter()-balls[1].getCenter());
    double d2 = cv::norm(balls[1].getCenter()-balls[2].getCenter());
    double d3 = cv::norm(balls[2].getCenter()-balls[0].getCenter());

    if((distances[0]<= d1 +20 && distances[0]>= d1-20)  && (distances[2]<= d2 +15 && distances[2]>= d2-20)){
        putText(img, "a" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("a");
    }
    else if (distances[0]<= d1 +20 && distances[0]>= d1-20  && distances[2]<= d3 +20 && distances[2]>= d3-20){
        putText(img, "a" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("a");
    }
    else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[2]<= d3 +20 && distances[2]>= d3-20){
        putText(img, "a" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("a");
    }

    else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[2]<= d1 +20 && distances[2]>= d1-20){
        putText(img, "a" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("a");
    }
    else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[2]<= d2 +20 && distances[2]>= d2-20){
        putText(img, "a" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("a");
    }
    else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[2]<= d1 +20 && distances[2]>= d1-20){
        putText(img, "a" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("a");
    }

    if((distances[0]<= d1 +20 && distances[0]>= d1-20)  && (distances[1]<= d2 +15 && distances[1]>= d2-20)){
        putText(img, "b" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("b");
    }
    else if (distances[0]<= d1 +20 && distances[0]>= d1-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
        putText(img, "b" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("b");
    }
    else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
        putText(img, "b" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("b");
    }

    else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
        putText(img, "b" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("b");
    }
    else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[1]<= d2 +20 && distances[1]>= d2-20){
        putText(img, "b" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("b");
    }
    else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
        putText(img, "b" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("b");
    }



    if((distances[2]<= d1 +20 && distances[2]>= d1-20)  && (distances[1]<= d2 +15 && distances[1]>= d2-20)){
        putText(img, "c" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("c");
    }
    else if (distances[2]<= d1 +20 && distances[2]>= d1-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
        putText(img, "c" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("c");
    }
    else if (distances[2]<= d2 +20 && distances[2]>= d2-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
        putText(img, "c" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("c");
    }

    else if (distances[2]<= d2 +20 && distances[2]>= d2-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
        putText(img, "c" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[1].setName("c");
    }
    else if (distances[2]<= d3 +20 && distances[2]>= d3-20  && distances[1]<= d2 +20 && distances[1]>= d2-20){
        putText(img, "c" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[2].setName("c");
    }
    else if (distances[2]<= d3 +20 && distances[2]>= d3-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
        putText(img, "c" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        balls[0].setName("c");
    }
}


