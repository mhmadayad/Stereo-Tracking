#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

int x = 0;

int main(int argc, char const *argv[])
{
  char* imgs_directory;
  char* extension;

  static struct poptOption options[] = {
    { "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory to save images in","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  VideoCapture cap1(0);
  VideoCapture cap2(1);
  Mat img1, img2;
  char ch = 0;
  while (true) {
    ch = 0;
    cap2 >> img1;
    cap1 >> img2;
    imshow("cap1",img1);
    imshow("cap2",img2);
    ch = cv::waitKey(30);
//    std::cout <<"not saved-" <<std::endl;
    if (ch==32) {//space key to save frame
      x++;
      std::cout <<"saved-"<< x <<std::endl;
      char filename1[200], filename2[200];
      sprintf(filename1, "%sleft%d.%s", imgs_directory, x, extension);
      sprintf(filename2, "%sright%d.%s", imgs_directory, x, extension);
      cout << "Saving img pair " << x << endl;
      imwrite(filename1, img1);
      imwrite(filename2, img2);
      //ch=0;
    }
    waitKey(30);
  }
  return 0;
}
