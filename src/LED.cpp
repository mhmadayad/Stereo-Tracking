#include "LED.h"
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
#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/core/utility.hpp>

using namespace std;

LED::LED(std::string name, double radius, cv::Point2f center,std::vector<cv::Point> contours)
      : name(name),radius(radius), center(center),contour(contours){
}

double LED::getRadius(){
    return radius;
}

std::string LED::getName(){
    return name;
}

cv::Point2f LED::getCenter(){
    return center;
}
std::vector<cv::Point> LED::getContour(){
    return contour;
}

void LED::setName(std::string name){
    this->name=name;
}



//LEDs detection
std::vector<LED> LED::findLEDs(cv::Mat frame, int scenario,int num){
    std::vector<LED> LEDs;
    cv::Mat grayImage , threshold;
    //cv::cvtColor( frame, grayImage, CV_BGR2GRAY );
    ///fix me
    frame.copyTo(grayImage);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    // Detect edges using Threshold
    cv::threshold( grayImage, threshold, 50, 255,0 );//cv::THRESH_BINARY );
    // Find contours
    if(num==1)
        cv::imshow("thresholdL",threshold);
    else
        cv::imshow("threhsoldR",threshold);
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
        //std::cout << radius[i] << std::endl;
        //find balls between min and max radius
        if((int)radius[i] >minRadius && (int)radius[i]<maxRadius){
            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            // std::cout << center[i] << std::endl;
            //balls bounding box
            //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            //draw circles around the balls
            cv::circle( mask, center[i], 10, CV_RGB(0,255,0), CV_FILLED, 8, 0 );
            cv::circle( drawing, center[i], 10, CV_RGB(0,255,0), 1, 8, 0 );
            LEDs.push_back(LED("LED",radius[i],cv::Point(int(center[i].x),int(center[i].y)),contours[i]));
        }
    }

    if(scenario==1){
         if(LEDs.size()== 3){
        //draw the triangle
            markLEDs(LEDs,drawing, scenario);
            //triange from the centers of three LEDs
            cv::line(drawing, LEDs[0].getCenter(), LEDs[1].getCenter(), CV_RGB(255,0,0),2);
            cv::line(drawing, LEDs[1].getCenter(), LEDs[2].getCenter(), CV_RGB(255,0,0),2);
            cv::line(drawing, LEDs[2].getCenter(), LEDs[0].getCenter(), CV_RGB(255,0,0),2);
            //markCircles(centers,drawing);

            //center of triangle
            int posX=( LEDs[0].getCenter().x + LEDs[1].getCenter().x +  LEDs[2].getCenter().x )/3;
            int posY=( LEDs[0].getCenter().y + LEDs[1].getCenter().y +  LEDs[2].getCenter().y )/3;
            cv::putText(drawing, "center", cv::Point(posX,posY), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);

        }
        else if(LEDs.size()>3)
            std::cout << "noise-> detected more than 3 balls" <<std::endl;
        else
            std::cout <<LEDs.size() << " balls detected" << std::endl;
    }

//    else if(scenario==2){
//        if(balls.size()== 2){
//            //draw the triangle
//            markBalls(balls,drawing,scenario);
//            //triange from the centers of three balls
//            cv::line(drawing, balls[0].getCenter(), balls[1].getCenter(), CV_RGB(255,0,0),2);
//            //markCircles(centers,drawing);
//        }

//        else if(balls.size()>2)
//            std::cout << "noise-> detected more than 3 balls" <<std::endl;
//        else
//            std::cout <<balls.size() << " detected" << std::endl;
//    }

    // Show in a window
    if(num==1)
        cv::imshow( "ContoursL", drawing );
    else
        cv::imshow( "ContoursR", drawing );
    //cv::imshow( "Contours", drawing );
    cv::cvtColor( mask, mask, CV_BGR2GRAY );
    mask.convertTo(mask,CV_8U);

    //cv::imshow( "mask", mask );
    return LEDs;
    //return balls;
}

//Mark balls (A,B,C) despite the transformation takes place in next frame
//tracking


void getLEDsRelPos(std::vector<LED> &LEDs){

    int posA=0;
    int posB=0;
    int posC=0;
    cv::Point c1=LEDs[0].getCenter();
    cv::Point c2=LEDs[1].getCenter();
    cv::Point c3=LEDs[2].getCenter();

    if(c1.y<c2.y && c1.y<c3.y){
        posA=0;
        if(c2.x<c3.x){
            posB=1;
            posC=2;
        }
        else{
            posB=2;
            posC=1;
            //std::swap(balls[1],balls[2]);
        }
    }
    else if(c2.y<c1.y && c2.y<c3.y){
        posA=1;
        if(c1.x<c3.x){
            posB=0;
            posC=2;
            //std::swap(balls[0],balls[1]);
        }
        else{
            posB=2;
            posC=0;
            //std::swap(balls[1],balls[0]);
            //std::swap(balls[1],balls[2]);
        }
    }

    else {
        posA=2;
        if(c1.x<c2.x){
            posB=0;
            posC=1;
            //std::swap(balls[0],balls[2]);
            //std::swap(balls[2],balls[1]);
        }
        else{
            posB=1;
            posC=0;
            //std::swap(balls[0],balls[2]);
        }
    }
//    pos.push_back(posA);
//    pos.push_back(posB);
//    pos.push_back(posC);

    LEDs[posA].setName("a");
    LEDs[posB].setName("b");
    LEDs[posC].setName("c");

}

///TODO:: A is always on top (smallest Y)
/// b and c smalles distance
/// A to b second smallest distance
void LED::markLEDs(std::vector<LED> &LEDs , cv::Mat &img,int scenario){
    //distances between the balls. i.e-> triangle sides

    if(scenario==1 && LEDs.size()==3){


//        double d1 = cv::norm(LEDs[0].getCenter()-LEDs[1].getCenter());
//        double d2 = cv::norm(LEDs[1].getCenter()-LEDs[2].getCenter());
//        double d3 = cv::norm(LEDs[2].getCenter()-LEDs[0].getCenter());



//        std::cout << d1 <<"-" <<d2 <<"-"<<d3<<std::endl;

        cv::Point c1=LEDs[0].getCenter();
        cv::Point c2=LEDs[1].getCenter();
        cv::Point c3=LEDs[2].getCenter();
        getLEDsRelPos(LEDs);
        putText(img, LEDs[0].getName() ,LEDs[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        putText(img, LEDs[1].getName() ,LEDs[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
        putText(img, LEDs[2].getName() ,LEDs[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);

//        if((distances[0]<= d1 +20 && distances[0]>= d1-20)  && (distances[2]<= d2 +15 && distances[2]>= d2-20)){
//            putText(img, "a" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("a");
//            //temp[0]=balls[1];


//        }
//        else if (distances[0]<= d1 +20 && distances[0]>= d1-20  && distances[2]<= d3 +20 && distances[2]>= d3-20){
//            putText(img, "a" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("a");
//            //temp[0]=balls[0];
//        }
//        else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[2]<= d3 +20 && distances[2]>= d3-20){
//            putText(img, "a" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("a");
//            //temp[0]=balls[2];
//        }

//        else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[2]<= d1 +20 && distances[2]>= d1-20){
//            putText(img, "a" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("a");
//            //temp[0]=balls[1];
//        }
//        else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[2]<= d2 +20 && distances[2]>= d2-20){
//            putText(img, "a" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("a");
//            //temp[0]=balls[2];
//        }
//        else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[2]<= d1 +20 && distances[2]>= d1-20){
//            putText(img, "a" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("a");
//            //temp[0]=balls[0];
//        }

//        if((distances[0]<= d1 +20 && distances[0]>= d1-20)  && (distances[1]<= d2 +15 && distances[1]>= d2-20)){
//            putText(img, "b" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("b");
//            //temp[1]=balls[1];
//        }
//        else if (distances[0]<= d1 +20 && distances[0]>= d1-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
//            putText(img, "b" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("b");
//            //temp[1]=balls[0];
//        }
//        else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
//            putText(img, "b" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("b");
//            //temp[1]=balls[2];
//        }

//        else if (distances[0]<= d2 +20 && distances[0]>= d2-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
//            putText(img, "b" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("b");
//            //temp[1]=balls[1];
//        }
//        else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[1]<= d2 +20 && distances[1]>= d2-20){
//            putText(img, "b" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("b");
//            //temp[1]=balls[2];
//        }
//        else if (distances[0]<= d3 +20 && distances[0]>= d3-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
//            putText(img, "b" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("b");
//            //temp[1]=balls[0];
//        }


//        if((distances[2]<= d1 +20 && distances[2]>= d1-20)  && (distances[1]<= d2 +15 && distances[1]>= d2-20)){
//            putText(img, "c" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("c");
//            //temp[2]=balls[1];
//        }
//        else if (distances[2]<= d1 +20 && distances[2]>= d1-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
//            putText(img, "c" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("c");
//            //temp[2]=balls[0];
//        }
//        else if (distances[2]<= d2 +20 && distances[2]>= d2-20  && distances[1]<= d3 +20 && distances[1]>= d3-20){
//            putText(img, "c" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("c");
//            //temp[2]=balls[2];
//        }

//        else if (distances[2]<= d2 +20 && distances[2]>= d2-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
//            putText(img, "c" , balls[1].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("c");
//            //temp[2]=balls[1];
//        }
//        else if (distances[2]<= d3 +20 && distances[2]>= d3-20  && distances[1]<= d2 +20 && distances[1]>= d2-20){
//            putText(img, "c" , balls[2].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[2].setName("c");
//            //temp[2]=balls[2];
//        }
//        else if (distances[2]<= d3 +20 && distances[2]>= d3-20  && distances[1]<= d1 +20 && distances[1]>= d1-20){
//            putText(img, "c" , balls[0].getCenter(), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("c");
//            //temp[2]=balls[0];
//        }

    }

//    else if(scenario==2 && balls.size()==2){

//        cv::Point c1=balls[0].getCenter();
//        cv::Point c2=balls[1].getCenter();
//        if(c1.y < c2.y){
//            putText(img, "a" , cv::Point(c1.x+5,c1.y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("a");
//            putText(img, "b" , cv::Point(c2.x+5,c2.y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("b");
//        }
//        else{
//            putText(img, "b" , cv::Point(c1.x+5,c1.y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[0].setName("b");
//            putText(img, "a" , cv::Point(c2.x+5,c2.y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);
//            balls[1].setName("a");
//            std::swap(balls[0],balls[1]);

//        }
//    }
}


