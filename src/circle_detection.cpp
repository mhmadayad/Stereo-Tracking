
#include <sstream>
#include <string>
#include <iostream>
#include "Constants.h"
#include "Ball.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/core/utility.hpp>
#include <a.out.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>


using namespace cv;
using namespace std;



//int main(int argc, char* argv[])
//{
//    VideoCapture capture;
//        //open capture
//        capture.open(0);
//       // capture1.open(0);
//        //set height and width of capture frame
//        //capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
//        //capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
//        waitKey(1000);

//    if (!capture.isOpened()) { //check if video device is connected
//    return -1;
//    }
//    while(1){

//      Mat gray;
//      Mat output;
//      Mat src;
//      capture >> src;

//      if(src.empty()){
//          return -1;
//      }

//      src.copyTo( output );
//      // Check if image is loaded fine

//      DetectCircles(src);
//      cvtColor(src, gray, COLOR_BGR2GRAY);
//      //DetectCircles(gray);
//      GaussianBlur(gray,gray,Size(3,3),0);
//      medianBlur(gray, gray, 1);
//      cv::Mat rangeRes = cv::Mat::zeros(gray.size(), CV_8UC1);

//     // threshold( gray, gray, 100, 255,0 );
//      cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
//      cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
//      std::vector<Vec3f> circles;

//      HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
//                   gray.rows/16,  // change this value to detect circles with different distances to each other
//                   100, 30, 0, 30 // change the last two parameters
//              // (min_radius & max_radius) to detect larger circles
//      );
//     //================================
////      if(circles.size()==3){
////        createShapeMask(binary , circles);
//      //================================

//      for( size_t i = 0; i < circles.size(); i++ )
//      {
//           std::cout << circles.size() << std::endl;
//           Vec3i c = circles[i];
//          //createShapeMask();
//          Point center = Point(c[0], c[1]);
//          // circle center
//          //circle( output, center, 1, Scalar(0,100,100), 3, LINE_AA);
//          // circle outline
//          double radius = c[2];
//          double rc=radius;
//          circle( output, center, radius, Scalar(255,0,255), 1 , LINE_AA);
//          rectangle(output,Point(center.x - rc, center.y - rc), Point(center.x + rc, center.y + rc), Scalar(255, 0, 0), 3, LINE_AA);
//          putText(output, "center", center, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
//      }
//      //imshow("s circles", gray);
//      imshow("detected circles", output);
//      imshow("gr",gray);
//      //cap.release();
//      waitKey(30);
//    }

//      capture.release();
//      return 0;
//}


using namespace std;

// > Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// < Color to be tracked

//int a(  )
//{

//    cv::Mat frame;

//    // >>>> Kalman Filter
//    int stateSize = 6;
//    int measSize = 4;
//    int contrSize = 0;

//    unsigned int type = CV_32F;
//    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

//    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    //cv::Mat procNoise(stateSize, 1, type)
//    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

//    // Transition State Matrix A
//    // [ 1 0 dT 0  0 0 ]
//    // [ 0 1 0  dT 0 0 ]
//    // [ 0 0 1  0  0 0 ]
//    // [ 0 0 0  1  0 0 ]
//    // [ 0 0 0  0  1 0 ]
//    // [ 0 0 0  0  0 1 ]
//    cv::setIdentity(kf.transitionMatrix);

//    // Measure Matrix H
//    // [ 1 0 0 0 0 0 ]
//    // [ 0 1 0 0 0 0 ]
//    // [ 0 0 0 0 1 0 ]
//    // [ 0 0 0 0 0 1 ]
//    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//    kf.measurementMatrix.at<float>(0) = 1.0f;
//    kf.measurementMatrix.at<float>(7) = 1.0f;
//    kf.measurementMatrix.at<float>(16) = 1.0f;
//    kf.measurementMatrix.at<float>(23) = 1.0f;

//    // Process Noise Covariance Matrix Q
//    // [ Ex   0   0     0     0    0  ]
//    // [ 0    Ey  0     0     0    0  ]
//    // [ 0    0   Ev_x  0     0    0  ]
//    // [ 0    0   0     Ev_y  0    0  ]
//    // [ 0    0   0     0     Ew   0  ]
//    // [ 0    0   0     0     0    Eh ]
//    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
//    kf.processNoiseCov.at<float>(0) = 1e-2;
//    kf.processNoiseCov.at<float>(7) = 1e-2;
//    kf.processNoiseCov.at<float>(14) = 5.0f;
//    kf.processNoiseCov.at<float>(21) = 5.0f;
//    kf.processNoiseCov.at<float>(28) = 1e-2;
//    kf.processNoiseCov.at<float>(35) = 1e-2;

//    // Measures Noise Covariance Matrix R
//    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
//    // <<<< Kalman Filter

//    // Camera Index
//    int idx = 0;

//    // Camera Capture
//    cv::VideoCapture cap;

//    // >>>>> Camera Settings
//    if (!cap.open(idx))
//    {
//        cout << "Webcam not connected.\n" << "Please verify\n";
//        return EXIT_FAILURE;
//    }

////    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
////    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
////    // <<<<< Camera Settings

//    cout << "\nHit 'q' to exit...\n";

//    char ch = 0;

//    double ticks = 0;
//    bool found = false;

//    int notFoundCount = 0;

//    // >>>>> Main loop
//    while (ch != 'q' && ch != 'Q')
//    {
//        double precTick = ticks;
//        ticks = (double) cv::getTickCount();

//        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

//        // Frame acquisition
//        cap >> frame;

//        cv::Mat res;
//        frame.copyTo( res );

//        if (found)
//        {
//            // >>>> Matrix A
//            kf.transitionMatrix.at<float>(2) = dT;
//            kf.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A

//            cout << "dT:" << endl << dT << endl;

//            state = kf.predict();
//            cout << "State post:" << endl << state << endl;

//            cv::Rect predRect;
//            predRect.width = state.at<float>(4);
//            predRect.height = state.at<float>(5);
//            predRect.x = state.at<float>(0) - predRect.width / 2;
//            predRect.y = state.at<float>(1) - predRect.height / 2;

//            cv::Point center;
//            center.x = state.at<float>(0);
//            center.y = state.at<float>(1);
//            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

//            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
//        }

////        // >>>>> Noise smoothing
////        cv::Mat blur;
////        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
////        // <<<<< Noise smoothing

////        // >>>>> HSV conversion
////        cv::Mat frmHsv;
////        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
////        // <<<<< HSV conversion

////        // >>>>> Color Thresholding
////        // Note: change parameters for different colors
////        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
////        cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
////                    cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
////        // <<<<< Color Thresholding

////        // >>>>> Improving the result
////        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
////        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
////        // <<<<< Improving the result

////        // Thresholding viewing
////        cv::imshow("Threshold", rangeRes);

////        // >>>>> Contours detection
////        vector<vector<cv::Point> > contours;
////        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
////                         CV_CHAIN_APPROX_NONE);
////        // <<<<< Contours detection

////        // >>>>> Filtering
////        vector<vector<cv::Point> > balls;
//        vector<cv::Rect> ballsBox;
//        vector<Vec3f> circles= findCircles(frame);
////        for (size_t i = 0; i < contours.size(); i++)
////        {
////            cv::Rect bBox;
////            bBox = cv::boundingRect(contours[i]);

////            float ratio = (float) bBox.width / (float) bBox.height;
////            if (ratio > 1.0f)
////                ratio = 1.0f / ratio;

////            // Searching for a bBox almost square
////            if (ratio > 0.75 && bBox.area() >= 400)
////            {
////                balls.push_back(contours[i]);
////                ballsBox.push_back(bBox);
////            }
////        }
//        // <<<<< Filtering

//        cout << "Bizzaaa:" << circles.size() << endl;

//        // >>>>> Detection result
//        for (size_t i = 0; i < circles.size(); i++)
//        {

//            Vec3i c = circles[i];
//            Point center = Point(c[0], c[1]);
//            double radius = c[2];
//            cv::Rect rRect(Point(c[0]-radius, c[1]-radius), Point(c[0] + radius, c[1]+radius));
//            ballsBox.push_back(rRect);
//            circle( res, center, radius, Scalar(255,0,255), 1 , LINE_AA);

////            cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
//            cv::rectangle(res, rRect, CV_RGB(0,255,0), 2);

////            cv::Point center;
////            center.x = ballsBox[i].x + ballsBox[i].width / 2;
////            center.y = ballsBox[i].y + ballsBox[i].height / 2;
////            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

//            stringstream sstr;
//            sstr << "(" << center.x << "," << center.y << ")";
//            cv::putText(res, sstr.str(),
//                        cv::Point(center.x, center.y),
//                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
//        }
//        // <<<<< Detection result

//        // >>>>> Kalman Update
//        if (circles.size() == 0)
//        {
//            notFoundCount++;
//            cout << "notFoundCount:" << notFoundCount << endl;
//            if( notFoundCount >= 100 )
//            {
//                found = false;
//            }
//            /*else
//                kf.statePost = state;*/
//        }
//        else
//        {
//            notFoundCount = 0;

//            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
//            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
//            meas.at<float>(2) = (float)ballsBox[0].width;
//            meas.at<float>(3) = (float)ballsBox[0].height;

//            if (!found) // First detection!
//            {
//                // >>>> Initialization
//                kf.errorCovPre.at<float>(0) = 1; // px
//                kf.errorCovPre.at<float>(7) = 1; // px
//                kf.errorCovPre.at<float>(14) = 1;
//                kf.errorCovPre.at<float>(21) = 1;
//                kf.errorCovPre.at<float>(28) = 1; // px
//                kf.errorCovPre.at<float>(35) = 1; // px

//                state.at<float>(0) = meas.at<float>(0);
//                state.at<float>(1) = meas.at<float>(1);
//                state.at<float>(2) = 0;
//                state.at<float>(3) = 0;
//                state.at<float>(4) = meas.at<float>(2);
//                state.at<float>(5) = meas.at<float>(3);
//                // <<<< Initialization

//                kf.statePost = state;

//                found = true;
//            }
//            else
//                kf.correct(meas); // Kalman Correction

//            cout << "Measure matrix:" << endl << meas << endl;
//        }
//        // <<<<< Kalman Update

//        // Final result
//        cv::imshow("Tracking", res);

//        // User key
//        ch = cv::waitKey(30);
//    }
//    // <<<<< Main loop

//    return EXIT_SUCCESS;
//}

int main(){

    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "myFunction() took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " ms" << std::endl;
    VideoCapture cap(0);
    VideoCapture cap1(1);
    Mat src;
    Mat src1;
    Mat gray;
    Mat Mask;
    bool timer=false;
    Ball bizza ("aaa", 1.0, Point2f(1.1));
    while(1){

        finish = std::chrono::high_resolution_clock::now();
        if(timer)
            std::cout << "myFunction() took "<< std::chrono::duration_cast<milli>(finish - start).count()<< " milliseconds\n";

        start = std::chrono::high_resolution_clock::now();
        timer=true;

        cap1 >> src;
        cap >> src1;

       // Convert image to gray and blur it
        cvtColor( src, gray, CV_BGR2GRAY );
        blur( gray, gray, Size(3,3) );
       // Create Window
        imshow( "sourceRight", src );
        imshow( "sourceLeft", src1 );
        std::vector<Ball> balls=bizza.findBalls(src,Mask);
       // imshow("mask",Mask);
        waitKey(30);
    }

    return(0);
}
