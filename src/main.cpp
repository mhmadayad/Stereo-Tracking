#include <iostream>
#include <fstream>
#include <string>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "StereoMatching.h"
#include "LED.h"




using namespace std;
using namespace cv;
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

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudA,
                                                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudB,
                                                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudC)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorA(cloudA, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorB(cloudB, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorC(cloudC, 0, 0, 255);

  viewer->addPointCloud<pcl::PointXYZ> (cloudA, single_colorA, "cloudA");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudA");

  viewer->addPointCloud<pcl::PointXYZ> (cloudB, single_colorB, "cloudB");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudB");
  viewer->addPointCloud<pcl::PointXYZ> (cloudC, single_colorC, "cloudC");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudC");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}

int main( int argc, char** argv )
{
    //testPlot();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptrA (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_colorA(cloud_ptrA, 255, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptrB (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_colorB(cloud_ptrB, 0, 255, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptrC (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_colorC(cloud_ptrC, 0, 0, 255);

    pcl::PointXYZ pointA,pointB,pointC;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = customColourVis(cloud_ptrA,cloud_ptrB,cloud_ptrC);

    int scenario=1; ///1 for three balls , 2 for two balls
    ///STEREORECTIFY DATA
    vector<Point> cntrs;
    LED L("led",0,Point2f(1,1),cntrs);
    Mat imgLeft, imgRight;
    Mat map11, map12, map21, map22;
    Mat img1r, img2r;
    ///for triangulate points algorithm
    vector<Point2d> srcPts;
    vector<Point2d> dstPts;
    vector<LED> srcLEDs;
    vector<LED> dstLEDs;
    Mat pnts3D;
    vector<LED> rightLEDs,leftLEDs;
    Size img_size;

    std::ofstream out;

    std::ofstream out1;

    std::ofstream out2;
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

    ///Depth map instance;
    DepthMap Depth(1);


    int64 t = getTickCount();
    int count = 0;
    double t1=0;
    bool recording=false;

    while (!viewer->wasStopped()) {


        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
        t1= t*1000/getTickFrequency();
        t = getTickCount();

        ///grab frames
        capturel >> imgLeft;
        capturer >> imgRight;
        if(imgLeft.empty() || imgRight.empty()){
            cout << "camera not connected" << endl;
            return -1;
        }

        cvtColor(imgLeft, imgLeft, CV_BGR2GRAY);
        cvtColor(imgRight, imgRight, CV_BGR2GRAY);
        imshow("left Image",imgLeft);
        imshow("right Image",imgRight);

        ///rectification
        ///FIXME
        img_size = imgLeft.size();
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
        remap(imgLeft, img1r, map11, map12, INTER_LINEAR);
        remap(imgRight, img2r, map21, map22, INTER_LINEAR);



        rightLEDs=L.findLEDs(imgRight,scenario,0);
        leftLEDs =L.findLEDs(imgLeft,scenario,1);
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
//                cout<<"pnts3dsize="<<pnts3D.size()<<endl;


                if(recording){
                    out.open ("data/point1.txt", std::ios::app);
                    out1.open("data/point2.txt", std::ios::app);
                    out2.open("data/point3.txt", std::ios::app);



                    //save points
                    out <<pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0) <<","
                    <<pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0) <<","
                    <<pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0) <<","<<t1<<"\n";

                    out1 <<pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1) <<","
                    <<pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1) <<","
                    <<pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1) <<","<<t1<<"\n";

                    out2 <<pnts3D.at<double>(0,2)/pnts3D.at<double>(3,2) <<","
                    <<pnts3D.at<double>(1,2)/pnts3D.at<double>(3,2) <<","
                    <<pnts3D.at<double>(2,2)/pnts3D.at<double>(3,2) <<"," <<t1<<"\n";

                    out.close();
                    out1.close();
                    out2.close();
                    cout <<"point1 3D: "<<pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0) <<","
                                           <<pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0) << ","<<
                                            pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0)<<","<<t1<<endl;

                    }

                    //update clouds for PCL visualization
                    pointA.x=pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0);
                    pointA.y=pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0);
                    pointA.z=pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0);
                    cloud_ptrA->points.push_back(pointA);

                    pointB.x=pnts3D.at<double>(0,1)/pnts3D.at<double>(3,1);
                    pointB.y=pnts3D.at<double>(1,1)/pnts3D.at<double>(3,1);
                    pointB.z=pnts3D.at<double>(2,1)/pnts3D.at<double>(3,1);
                    cloud_ptrB->points.push_back(pointB);

                    pointC.x=pnts3D.at<double>(0,2)/pnts3D.at<double>(3,2);
                    pointC.y=pnts3D.at<double>(1,2)/pnts3D.at<double>(3,2);
                    pointC.z=pnts3D.at<double>(2,2)/pnts3D.at<double>(3,2);
                    cloud_ptrC->points.push_back(pointC);

               //clear points after each frame
               srcLEDs.clear();
               dstLEDs.clear();
               srcPts.clear();
               dstPts.clear();

               count++;


          }


        }

        viewer->updatePointCloud(cloud_ptrA,cloud_colorA,"cloudA");
        viewer->updatePointCloud(cloud_ptrB,cloud_colorB,"cloudB");
        viewer->updatePointCloud(cloud_ptrC,cloud_colorC,"cloudC");
        viewer->spinOnce (10);

        char c = (char)waitKey(30);
        if (c == 32){//space key
            recording=true;
        }

        else if(c==27){
            break;
        }

        t = getTickCount() - t;

    }


    destroyAllWindows();
    return EXIT_SUCCESS;
}
