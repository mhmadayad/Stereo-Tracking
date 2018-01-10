
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <stdio.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


using namespace cv;
using namespace std;


static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

int main(int argc, const char** argv)
{

 // add your file name
 VideoCapture cap(1);


 Mat flow, frame;
         // some faster than mat image container
 UMat  flowUmat, prevgray;

 for (;;)
 {

  bool Is = cap.grab();
  if (Is == false) {
                         // if video capture failed
   cout << "Video Capture Fail" << endl;
   break;
  }
  else {



   Mat img;
   Mat original;
   Mat cflow;
   Mat flow;
   // capture frame from video file
   cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
   resize(img, img, Size(640, 480));

   // save original for later
   img.copyTo(original);

   // just make current frame gray
   cvtColor(img, img, COLOR_BGR2GRAY);


   // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous                           //and current frame
   //if there is no current frame
   // go to this part and fill previous frame
   //else {
   // img.copyTo(prevgray);
   //   }
   // if previous frame is not empty.. There is a picture of previous frame. Do some                                  //optical flow alg.

   if (prevgray.empty() == false ) {

    // calculate optical flow
    calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.8, 1, 12, 2, 5, 1.1, 0);
    cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
    imshow("flow", cflow);
    flowUmat.copyTo(flow);

           // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < original.rows; y += 5) {
     for (int x = 0; x < original.cols; x += 5)
     {
              // get the flow from y, x position * 10 for better visibility
              const Point2f flowatxy = flow.at<Point2f>(y, x);
                     // draw line at flow direction
       line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,0,0));
                                                         // draw initial point
       circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);


      }

     }

                                 // draw the results
    namedWindow("prew", WINDOW_AUTOSIZE);
    imshow("prew", original);

                                 // fill previous image again
    img.copyTo(prevgray);

   }
   else {

                                 // fill previous image in case prevgray.empty() == true
    img.copyTo(prevgray);

   }


   int key1 = waitKey(20);

  }
 }
}
