#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Ball.h"
#include "StereoMatching.h"
#include "popt_pp.h"
#include <fstream>
#include "matplotlibcpp.h"
#include <string>
#include "LED.h"
using namespace std;
using namespace cv;
namespace plt = matplotlibcpp;
///Save 3D points out of disparity map
static void saveXYZ(const char* filename, const Mat& mat , vector<Point2d> centers)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            for(int i=0 ; i<centers.size();i++){
                if(centers[i].x==x && centers[i].y==y){
                Vec3f point = mat.at<Vec3f>(y, x);

                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
                cout <<"("<< point[0] <<"," <<point[1] << "," << point[2] <<")" <<endl;
                }
            }
        }
    }
    fclose(fp);
}

void testPlot(){
    int n = 1000;
        std::vector<double> x, y, z;

        for(int i=0; i<n; i++) {
            x.push_back(i);
            y.push_back(i*2);
            //z.push_back(log(i));
            string a = "z";
            string b = "z";
            if (i % 10 == 0) {
                // Clear previous plot
                plt::clf();
                // Plot line from given x and y data. Color is selected automatically.
                plt::plot(x, y);
                plt::subplot(1,1,1);
                //plt::plot(x,y,std::make_pair(a,b));
                // Plot a line whose name will show up as "log(x)" in the legend.
                plt::named_plot("log(x)", x, z);

                // Set x-axis to interval [0,1000000]
                plt::xlim(0, 1000);
                plt::ylim(0,2000);

                // Add graph title
                plt::title("Sample figure");
                // Enable legend.
                plt::legend();
                // Display plot continuously
                plt::pause(0.0000001);
            }
        }
}

void reprojectImage3D( InputArray _disparity,
                            OutputArray __3dImage, InputArray _Qmat,
                             bool handleMissingValues, int dtype,vector<Point2d> centers)
{

    Mat disparity = _disparity.getMat(), Q = _Qmat.getMat();
    int stype = disparity.type();

    CV_Assert( stype == CV_8UC1 || stype == CV_16SC1 ||
               stype == CV_32SC1 || stype == CV_32FC1 );
    CV_Assert( Q.size() == Size(4,4) );

    dtype = CV_32FC3;

    __3dImage.create(disparity.size(), CV_MAKETYPE(dtype, 3));
    Mat _3dImage = __3dImage.getMat();

    const double bigZ = 10000.;
    Matx44d _Q;
    Q.convertTo(_Q, CV_64F);

    int x, cols = disparity.cols;
    CV_Assert( cols >= 0 );

    vector<float> _sbuf(cols);
    vector<Vec3f> _dbuf(cols);
    float* sbuf = &_sbuf[0];
    Vec3f* dbuf = &_dbuf[0];
    double minDisparity = FLT_MAX;

    // NOTE: here we quietly assume that at least one pixel in the disparity map is not defined.
    // and we set the corresponding Z's to some fixed big value.
    if( handleMissingValues )
        cv::minMaxIdx( disparity, &minDisparity, 0, 0, 0 );

    for( int y = 0; y < disparity.rows; y++ )
    {
        float* sptr = sbuf;
        Vec3f* dptr = dbuf;



        const short* sptr0 = disparity.ptr<short>(y);
        for( x = 0; x < cols; x++ )
            sptr[x] = (float)sptr0[x];

        dptr = _3dImage.ptr<Vec3f>(y);

        for( x = 0; x < cols; x++)
        {
            for(int i = 0 ;i<centers.size(); i++)
            {
                if(centers[i].x==x && centers[i].y==y){
                    cout << "x:"<<centers[i].x << "y:"<<centers[i].y << endl;
                    double d = sptr[x];
                    Vec4d homg_pt = _Q*Vec4d(x, y, d, 1.0);
                    dptr[x] = Vec3d(homg_pt.val);
                    dptr[x] /= homg_pt[3];
                    if( fabs(d-minDisparity) <= FLT_EPSILON )
                        dptr[x][2] = bigZ;
                }
            }
        }
    }
}

void getLEDsCenters(vector<LED> b , vector<Point2d> &centers){

    for(int i=0;i<b.size();i++){

        centers.push_back(b.at(i).getCenter());
    }
}

int main( int argc, char** argv )
{
    //testPlot();
    FILE* fp = fopen("testData", "wt");
    std::ofstream outfile;
    outfile.open("fiftymicro/test.txt", std::ios_base::app);
    int scenario=1; ///1 for three balls , 2 for two balls
    int algorithm = 3;///1 for stereoBM , 2 for stereoSGBM , 3 for Triangulation
    ///STEREORECTIFY DATA
    vector<Point> cntrs;
    LED L("led",0,Point2f(1,1),cntrs);
    Mat xyz,disp8;
    Mat imgLeft, imgRight, disp_out;
    Mat map11, map12, map21, map22;
    Mat img1r, img2r, img1r_small, img2r_small;
    Mat imgDisparity16S, imgDisparity8U;
    ///for triangulate points algorithm
    vector<Point2d> srcPts;
    vector<Point2d> dstPts;
    vector<Point2d> centers;
    vector<LED> srcLEDs;
    vector<LED> dstLEDs;
    Mat pnts3D;

    ///get calibration params
    string calibration_filenameext = "extrinsics.yml";
    string calibration_filenameint = "intrinsics.yml";

    FileStorage fsext(calibration_filenameext, FileStorage::READ);
    FileStorage fsint(calibration_filenameint, FileStorage::READ);
    if(!fsext.isOpened() || !fsint.isOpened())
    {
        //printf("Failed to open file %s\n", calibration_filename.c_str());
        return -1;
    }

    Mat M1, D1, M2, D2,R, T, R1, P1, R2, P2, Q;
    fsint["M1"] >> M1;
    fsint["D1"] >> D1;
    fsint["M2"] >> M2;
    fsint["D2"] >> D2;
    fsext["R"] >> R;
    fsext["T"] >> T;
    fsext["R1"] >> R1;
    fsext["R2"] >> R2;
    fsext["P1"] >> P1;
    fsext["P2"] >> P2;
    fsext["Q"] >> Q;

    ///start video capture
    VideoCapture capturel(0);
    VideoCapture capturer(1);
    waitKey(2000);
    capturel >> imgRight;
    capturer >> imgLeft;
    waitKey(2000);
    int SADWindowSize = 5; /// block window size
    ///Call the constructor for StereoBM
    Ptr<StereoBM> sbm = StereoBM::create( 16, SADWindowSize );
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);


    ///Depth map instance;
    DepthMap Depth(1);


    int64 t = getTickCount();
    int count =0;
    while (true) {


        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
        t = getTickCount();

        ///grab frames
        capturel >> imgLeft;
        capturer >> imgRight;
        if(imgLeft.empty() | imgRight.empty()){
            cout << "camera not connected" << endl;
            return -1;
        }
        ///SHRINK AND FILTER INPUT IMAGE
//        pyrDown(imgLeft, imgLeft);
//        pyrDown(imgRight, imgRight);
//        bilateralFilter(imgLeftIN, imgLeft, 15, 150, 150);
//        bilateralFilter(imgRightIN, imgRight, 15, 150, 150);

        ///DISP MAT AND CONVERT TO GRAYSCALE
        cvtColor(imgLeft, imgLeft, CV_BGR2GRAY);
        cvtColor(imgRight, imgRight, CV_BGR2GRAY);
        imshow("left Image",imgLeft);
        imshow("right Image",imgRight);

        ///rectification
        Size img_size = imgLeft.size();
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
        remap(imgLeft, img1r, map11, map12, INTER_LINEAR);
        remap(imgRight, img2r, map21, map22, INTER_LINEAR);

        ///find balls in both frames
        ///for matching algorithms
        //vector<LED> leftLEDs;
//        if (algorithm==1 || algorithm==2){
//            ///Balls detection and Mask generation
//            ///FIXME: use rectified pair

//            imgDisparity16S= Mat( imgLeft.rows, imgLeft.cols, CV_16U );
//            imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8SC1 );

//            ///SHOW RECTIFIED
//            pyrDown(img1r, img1r_small);
//            pyrDown(img2r, img2r_small);
//            imshow("rectified left", img1r);
//            imshow("rectified right", img2r);
//            leftLEDs=L.findLEDs(img1r,scenario);
//            Mat mask=leftB.second;
//            ///COMPUTE DISPARITY
//            ///TODO :: compute(img1r,img2r);
//            ///FIX ME :: rectification output
//            if(algorithm==1)
//                imgDisparity16S=Depth.stereoBM(img1r,img2r,sbm);
//            else if(algorithm==2)
//                imgDisparity16S=Depth.stereoSGBM(img1r,img2r,sgbm);

//            ///-- Check its extreme values
//            double minVal; double maxVal;
//            minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//    //      printf("Min disp: %f Max value: %f \n", minVal, maxVal);

//            imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
//    //      normalize(imgDisparity16S,imgDisparity8U,0,255,CV_MINMAX,CV_8U);
//            imgDisparity16S.convertTo(disp8, CV_8U);
//            imshow("disparity",disp8);
//            Mat out=mask.mul(disp8);
//            if(leftB.first.size()>0){
//                getBallCenters(leftB.first,centers);
//                reprojectImage3D(imgDisparity16S, xyz, Q,true,-1,centers);
//                saveXYZ("depth", xyz,centers);
//            }
//            imshow("MASK x DISP",out);

//            /// keep only z values
//    //        vector<Mat> channels(3);
//    //        split(xyz, channels);
//    //        disp_out = channels[2];
//    //        disp_out.convertTo(disp_out, CV_8UC1);
//    //        applyColorMap(disp_out, disp_out, 2);
//    //        imshow("Depth Z",disp_out);

//            pyrDown(imgDisparity8U, imgDisparity8U);
//            applyColorMap(imgDisparity8U, imgDisparity8U, 2);
//            namedWindow( "Disparity ColoredMap", WINDOW_NORMAL );
//            imshow( "Disparity ColoredMap", imgDisparity8U );
//            centers.clear();

//        }


        ///triangulatePoints
        //else{

            vector<LED> rightLEDs=L.findLEDs(imgRight,scenario,0);
            vector<LED> leftLEDs =L.findLEDs(imgLeft,scenario,1);
            if(scenario==1){

                if(leftLEDs.size()==3 && rightLEDs.size()==3){
                    srcLEDs=leftLEDs;
                    dstLEDs=rightLEDs;
                    for(int i=0; i<srcLEDs.size();i++){
                        srcPts.push_back(srcLEDs.at(i).getCenter());
                        dstPts.push_back(dstLEDs.at(i).getCenter());
                        std::cout<< srcLEDs.at(i).getCenter() <<std::endl;
                        std::cout<< dstLEDs.at(i).getCenter() <<std::endl;
                    }

                    pnts3D=Depth.triangulatePts(srcPts,dstPts,P1,P2);
                    cout<<"pnts3dsize="<<pnts3D.size()<<endl;
                    outfile.open("fiftymicro/test.txt", std::ios_base::app);/*
                    outfile <<pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1) <<","
                        <<pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1) <<","
                        <<pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1) <<"\n";*/

                    cout <<"point1 3D: "<<pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0) <<","
                                       <<pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0) << ","<<
                                        pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0)<<endl;

                    cout <<"point2 3D: " <<pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1) <<"," <<
                                         pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1) <<"," <<
                                         pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1)<<endl;

                    cout <<"point3 3D: " <<pnts3D.at<double>(0,2)/pnts3D.at<double>(3,2) <<"," <<
                                         pnts3D.at<double>(1,2)/pnts3D.at<double>(3,2) <<"," <<
                                         pnts3D.at<double>(2,2)/pnts3D.at<double>(3,2)<<endl;
//                    cout <<"point1 2D: "<<srcLEDs.at(0).getCenter() <<"," <<dstLEDs.at(0).getCenter()<<endl;
//                    cout <<"point2 2D: "<<srcLEDs.at(1).getCenter() <<"," <<dstLEDs.at(1).getCenter()<<endl;
//                    cout <<"point3 2D: "<<srcLEDs.at(2).getCenter() <<"," <<dstLEDs.at(2).getCenter()<<endl;

                    srcLEDs.clear();
                    dstLEDs.clear();
                    srcPts.clear();
                    dstPts.clear();


                }
            }
            ///scenario = 2 balls
//            else{
//                if(leftB.first.size() ==2 && rightB.first.size()==2){
//                    count++;
//                    srcLEDs=leftLEDs;
//                    dstLEDs=rightLEDs;
//                    for(int i=0; i<srcLEDs.size();i++){
//                        srcPts.push_back(LED(srcLEDs.at(i)).getCenter());
//                        dstPts.push_back(LED(dstLEDs.at(i)).getCenter());
//                    }
//                    pnts3D=Depth.triangulatePts(srcPts,dstPts,P1,P2);
//                    outfile.open("fiftymicro/test.txt", std::ios_base::app);
//                    outfile <<pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1) <<","
//                            <<pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1) <<","
//                            <<pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1) <<"\n";
//                    outfile.close();

//                    cout <<"point1 2D: "<<srcBall.at(0).getCenter() <<"," <<dstBall.at(0).getCenter()<<endl;
//                    cout <<"point1 3D: "<<pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0) <<","
//                      <<pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0) << ","<<
//                           pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0)<<endl;
//                    cout << " " <<endl;
//                    cout <<"point2 2D: "<<srcBall.at(1).getCenter() <<"," <<dstBall.at(1).getCenter()<<endl;
//                    cout <<"point2 3D: " <<pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1) <<"," <<
//                        pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1) <<"," <<
//                           pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1)<<endl;
//                    srcPts.clear();
//                    dstPts.clear();
//                    srcBall.clear();
//                    srcPts.clear();
//                    dstPts.clear();
//                    cout << " " <<endl;
//                }
//            }

//        }

        char c = (char)waitKey(30);
        if (c == 27){
            break;
        }
         t = getTickCount() - t;

    }
    destroyAllWindows();

    return EXIT_SUCCESS;
}







