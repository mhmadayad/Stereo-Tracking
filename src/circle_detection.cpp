
#include <sstream>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>


using namespace cv;

int main(int argc, char* argv[])
{


      // Loads an image
      Mat src = imread( "/home/moh/Pictures/Webcam/best2.jpg", IMREAD_COLOR );
      // Check if image is loaded fine
      if(src.empty()){
          return -1;
      }
      Mat gray;
      cvtColor(src, gray, COLOR_BGR2GRAY);
      medianBlur(gray, gray, 5);
      std::vector<Vec3f> circles;
      HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                   gray.rows/16,  // change this value to detect circles with different distances to each other
                   100, 30, 0, 0 // change the last two parameters
              // (min_radius & max_radius) to detect larger circles
      );
      for( size_t i = 0; i < circles.size(); i++ )
      {
          Vec3i c = circles[i];
          Point center = Point(c[0], c[1]);
          // circle center
          circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
          // circle outline
          int radius = c[2];
          circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
      }
      imshow("detected circles", src);
      waitKey();
      return 0;
}
