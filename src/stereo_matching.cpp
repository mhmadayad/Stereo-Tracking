/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>

#include "popt_pp.h"
#include "Constants.h"
#include "Ball.h"

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <ctime>
using namespace cv;
using namespace std;


Mat Mask;

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

static void getCameraMatrix(){

}


vector<Vec3f> findCircles(Mat Image)
{
    Mat grayImage;
    cvtColor( Image, grayImage, CV_BGR2GRAY );
    vector<Vec3f> circles;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using Threshold
    threshold( grayImage, threshold_output, 100, 255, THRESH_BINARY );
    // Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<Point2f>centers;
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ){

       approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }


  /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    Mat mask = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
        if((int)radius[i] >20 && (int)radius[i]<50){
           //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
           //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            circle( mask, center[i], (int)radius[i], CV_RGB(0,255,0), CV_FILLED, 8, 0 );
            circle( drawing, center[i], (int)radius[i], CV_RGB(0,255,0), 2, 8, 0 );
            Vec3i c(center[i].y,center[i].x,radius[i]);
            centers.push_back(center[i]);
            circles.push_back(c);
        }

    }
    if(centers.size()==3){
      //draw the triangle

      line(drawing, centers[0], centers[1], CV_RGB(255,0,0),2);
      line(drawing, centers[1], centers[2], CV_RGB(255,0,0),2);
      line(drawing, centers[2], centers[0], CV_RGB(255,0,0),2);
      //markCircles(centers,drawing);

      //center of triangle
      int posX=( centers[0].x + centers[1].x + centers[2].x )/3;
      int posY=( centers[0].y + centers[1].y + centers[2].y )/3;
      putText(drawing, "a", Point(posX,posY), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 1.0);

    }

  /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
    cvtColor( mask, mask, CV_BGR2GRAY );
 // imshow( "mask", mask );
    mask.copyTo(Mask);

    return circles;
}

int main(int argc, char** argv)
{


    //char* leftImage;
    //char* rightImage;
    char* disparity_filename;
    std::string point_cloud_filename;
    char* camSettings;

    static struct poptOption options[] = {
      //{ "leftimg",'L',POPT_ARG_STRING,&leftImage,0,"Directory containing left images","STR" },
      //{ "rightimg",'R',POPT_ARG_STRING,&rightImage,0,"Directory containing right images","STR" },
      { "disparityfile",'D',POPT_ARG_STRING,&disparity_filename,0,"Right image prefix","STR" },
      { "out_file",'o',POPT_ARG_STRING,&point_cloud_filename,0,"Output calibration filename (YML)","STR" },
      { "camsetting",'i',POPT_ARG_STRING,&camSettings,0,"Left camera calibration","STR" },
      //{ "extrinsics",'e',POPT_ARG_STRING,&rightcalib_file,0,"Right camera calibration","STR" },

      POPT_AUTOHELP
      { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    FileStorage fs(camSettings, FileStorage::READ);


    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;

    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);

    numberOfDisparities = 16;//parser.get<int>("max-disparity");
    SADWindowSize = 15;//parser.get<int>("blocksize");
    scale = 1;//parser.get<float>("scale");
    //no_display = parser.has("no-display");


    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }

    int color_mode = alg == STEREO_BM ? 0 : -1;
    Mat img1 = imread("/home/moh/Desktop/left.jpg", color_mode);
    Mat img2 = imread("/home/moh/Desktop/right.jpg", color_mode);
    vector<Vec3f> circles=findCircles(img1);

    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

//    if (scale != 1.f)
//    {
//        Mat temp1, temp2;
//        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//        resize(img1, temp1, Size(), scale, scale, method);
//        img1 = temp1;
//        resize(img2, temp2, Size(), scale, scale, method);
//        img2 = temp2;
//    }

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;


    Mat M1, D1, M2, D2;
    Mat R, T, R1, P1, R2, P2;
    fs["K1"] >> M1;
    fs["D1"] >> D1;
    fs["K2"] >> M2;
    fs["D2"] >> D2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["Q"] >> Q;
     M1 *= scale;
     M2 *= scale;

        //M1 Cam1 , D1 distortion
     stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
//        Mat map11, map12, map21, map22;

//        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

//        Mat img1r, img2r;
//        remap(img1, img1r, map11, map12, INTER_LINEAR);
//        remap(img2, img2r, map21, map22, INTER_LINEAR);

//        img1 = img1r;
//        img2 = img2r;

    std::cout << "asdasadasdb bizza" << std::endl;
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;


    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(21   );

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(48);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(20);
    sgbm->setDisp12MaxDiff(20);
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Mat disp, disp8;
    Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    /*if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
     */

    sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

//    disp = dispp.colRange(numberOfDisparities, img1p.cols);
//    if( alg != STEREO_VAR )
//        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
//    else
//
    Mat output;
    disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        output = disp8.mul(Mask);
        imshow("Mask",Mask);
        imshow("output",output);
        fflush(stdout);
        waitKey();
    }

    //imwrite(disparity_filename, disp8);


    printf("storing the point cloud...");
    fflush(stdout);
    Mat xyz;

    reprojectImageTo3D(output, xyz, Q, true);
    saveXYZ(point_cloud_filename.c_str(), xyz);
    printf("\n");


    return 0;
}
