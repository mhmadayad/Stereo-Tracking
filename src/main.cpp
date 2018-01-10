#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Ball.h"
#include "StereoMatching.h"
using namespace std;
using namespace cv;

///Save 3D points out of disparity map
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

    std::vector<float> _sbuf(cols);
    std::vector<Vec3f> _dbuf(cols);
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
            double d = sptr[x];
            Vec4d homg_pt = _Q*Vec4d(x, y, d, 1.0);
            dptr[x] = Vec3d(homg_pt.val);
            dptr[x] /= homg_pt[3];
            if( fabs(d-minDisparity) <= FLT_EPSILON )
                dptr[x][2] = bigZ;
            }

    }
}

void getBallCenters(vector<Ball> b , vector<Point2d> &centers){

    for(int i=0;i<b.size();i++){

        centers.push_back(b.at(i).getCenter());
    }
}

int main( int argc, char** argv )
{
    int scenario=2; ///1 for three balls , 2 for two balls
    int algorithm = 1;///1 for stereoBM , 2 for stereoSGBM
    ///STEREORECTIFY DATA
    Ball b("balls",0,Point2f(1,1));
    Mat xyz,disp8;
    Mat imgLeft, imgRight, disp_out;
    Mat map11, map12, map21, map22;
    Mat img1r, img2r, img1r_small, img2r_small;
    Mat imgDisparity16S, imgDisparity8U;
    ///for triangulate points algorithm
    vector<Point2d> srcPts;
    vector<Point2d> dstPts;
    vector<Point2d> centers;
    vector<Ball> srcBall;
    vector<Ball> dstBall;
    Mat pnts3D;

    ///get calibration params
    string calibration_filename = "cam_stereo.yml";
    FileStorage fs(calibration_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", calibration_filename.c_str());
        return -1;
    }

    Mat M1, D1, M2, D2,R, T, R1, P1, R2, P2, Q;
    fs["K1"] >> M1;
    fs["D1"] >> D1;
    fs["K2"] >> M2;
    fs["D2"] >> D2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

    ///start video capture
    VideoCapture capturel(0);
    VideoCapture capturer(1);


    int SADWindowSize = 5; /// block window size
    ///Call the constructor for StereoBM
    Ptr<StereoBM> sbm = StereoBM::create( 16, SADWindowSize );
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);


    ///Depth map instance;
    DepthMap Depth(1);


    int64 t = getTickCount();
    while (1) {

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
        pyrDown(imgLeft, imgLeft);
        pyrDown(imgRight, imgRight);
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
        std::pair<vector<Ball>,Mat> leftB=b.findBalls(imgLeft,scenario);
        std::pair<vector<Ball>,Mat> rightB=b.findBalls(imgRight,scenario);
        if (algorithm==1 || algorithm==2){
            ///Balls detection and Mask generation
            ///FIXME: use rectified pair

            Mat mask=leftB.second;
            imgDisparity16S= Mat( imgLeft.rows, imgLeft.cols, CV_16U );
            imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8SC1 );

            ///SHOW RECTIFIED
            pyrDown(img1r, img1r_small);
            pyrDown(img2r, img2r_small);
            imshow("rectified left", img1r);
            imshow("rectified right", img2r);

            ///COMPUTE DISPARITY
            ///TODO :: compute(img1r,img2r);
            ///FIX ME :: rectification output
            if(algorithm==1)
                imgDisparity16S=Depth.stereoBM(imgLeft,imgRight,sbm);
            else if(algorithm==2)
                imgDisparity16S=Depth.stereoSGBM(imgLeft,imgRight,sgbm);

            ///-- Check its extreme values
            double minVal; double maxVal;
            minMaxLoc( imgDisparity16S, &minVal, &maxVal );
    //      printf("Min disp: %f Max value: %f \n", minVal, maxVal);

            imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
    //      normalize(imgDisparity16S,imgDisparity8U,0,255,CV_MINMAX,CV_8U);
            imgDisparity16S.convertTo(disp8, CV_8U);
            imshow("disparity",disp8);
            Mat out=mask.mul(disp8);
            if(leftB.first.size()>0){
                getBallCenters(leftB.first,centers);
                reprojectImage3D(imgDisparity16S, xyz, Q,true,-1,centers);
                saveXYZ("depth", xyz);
            }
            imshow("MASK x DISP",out);

            /// keep only z values
    //        vector<Mat> channels(3);
    //        split(xyz, channels);
    //        disp_out = channels[2];
    //        disp_out.convertTo(disp_out, CV_8UC1);
    //        applyColorMap(disp_out, disp_out, 2);
    //        imshow("Depth Z",disp_out);

            pyrDown(imgDisparity8U, imgDisparity8U);
            applyColorMap(imgDisparity8U, imgDisparity8U, 2);
            namedWindow( "Disparity ColoredMap", WINDOW_NORMAL );
            imshow( "Disparity ColoredMap", imgDisparity8U );
            centers.clear();
        }


        ///triangulatePoints
        else{


            if(scenario==1){

                if(leftB.first.size()==3 && rightB.first.size()==3){
                    srcBall=leftB.first;
                    dstBall=rightB.first;
                    for(int i=0; i<srcBall.size();i++){
                        srcPts.push_back(Ball(srcBall.at(i)).getCenter());
                        dstPts.push_back(Ball(dstBall.at(i)).getCenter());
                    }

                    pnts3D=Depth.triangulatePts(srcPts,dstPts,P1,P2);
                }
            }
            ///scenario = 2 balls
            else{
                if(leftB.first.size() == rightB.first.size()==2){
                    srcBall=leftB.first;
                    dstBall=rightB.first;
                    for(int i=0; i<srcBall.size();i++){
                        srcPts.push_back(Ball(srcBall.at(i)).getCenter());
                        dstPts.push_back(Ball(dstBall.at(i)).getCenter());
                    }
                    pnts3D=Depth.triangulatePts(srcPts,dstPts,P1,P2);
                }
            }
        }

        char c = (char)waitKey(5);
        if (c == 27){
            break;
        }
         t = getTickCount() - t;

    }

    destroyAllWindows();
    return 0;
}









