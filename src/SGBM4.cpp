/*
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <ctime>
*/
#include "standard_includes.h"
#include "write_ply.h"
#include "scale_image.h"
#include "feature_match.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    //--------------------------------INITIATE VARIABLES----------------------------------

    float u;
    float v;

    vector<Point3f> list_points3d;
    vector<Point2f> list_points2d;
    vector<KeyPoint> keypoints_im2;

    const char* point_cloud_filename = 0;
    const char* point_cloud_filename_2 = 0;
    int scale_factor=4;

    point_cloud_filename = "cloud.ply";

    std::vector< DMatch > good_matches;

    Mat disp, disp8;

    Mat descriptors_1, descriptors_2;

    Mat xyz, inliers;

    bool flags = 1;
    Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);
    Mat rvec = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);

    Mat R_matrix = Mat::zeros(3, 3, CV_64FC1);
    Mat t_matrix = Mat::zeros(3, 1, CV_64FC1);

    bool useExtrinsicGuess = false;
    int iterationsCount=5000;
    float reprojectionError=5.0;
    double confidence=0.99;


    Mat jacobian;
    double aspectRatio = 0;
    vector<Point2f> imagePoints; 

    Point pt;
    vector<KeyPoint> keypoints_projected;

   //----------------------------------CONFIGURE STEREO AND CAMERA SETTINGS----------------------------------

    //assign SGBM because it gives superior results
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;

    int color_mode = alg == STEREO_BM ? 0 : -1;

    int SADWindowSize = 11;
    bool no_display;
    float scale;

    int minDisparity = 80;
    int numberOfDisparities = 224;
    int uniquenessRatio = 10;
    int speckleWindowSize = 10;
    int speckleRange = 1;
    int disp12MaxDiff = 10;
    int P1 = pow(8*3*SADWindowSize,2);
    int P2 = pow(32*3*SADWindowSize,2);
    bool fullDP=false;
    int preFilterCap = 0;
    int mode;

    float cx = 1688/scale_factor;
    float cy = 1352/scale_factor;
    float f = 2421/scale_factor;
    float T = -.0914*scale_factor; 

    // M1 is the camera intrinsics mof the left camera (known)
    Mat M1 = (Mat_<double>(3,3) << 2421.988247695817/scale_factor, 0.0, 1689.668741757609/scale_factor, 0.0, 2424.953969600827/scale_factor, 1372.029058638022/scale_factor, 0.0, 0.0, 1.0);

    int minHessian = 200; //the hessian affects the number of keypoints

    Mat Q = (Mat_<double>(4,4) << -1,0,0,cx,0,1,0,-cy,0,0,0,f,0,0,1/T,0);

    //------------------------------------LOAD IMAGES-----------------------------------------

    //The first two images are the stereo pair. The third image is the one that has moved (we don't know its position)
    std::string img1_filename = "../Sorghum_Stitching/images/left000007.jpg";
    std::string img2_filename = "../Sorghum_Stitching/images/right000007.jpg";
    std::string img3_filename = "../Sorghum_Stitching/images/left000008.jpg";

    //-------------------------------------------SCALE IMAGES-------------------------------------------

    scaleImage scaledown; //(img1_filename, color_mode);
    scaleImage scaledown2; //(img3_filename, color_mode);
    scaleImage scaledown3; //(img2_filename, color_mode);

    Mat img1 = scaledown.downSize(img1_filename, color_mode);
    Mat img2 = scaledown2.downSize(img2_filename, color_mode);
    Mat img3 = scaledown3.downSize(img3_filename, color_mode);

    Size img_size = img1.size();

    //------------------------------------FIND MATCHES-----------------------------------------

    //start a timer just for kicks
    clock_t start;
    double duration;
    double feat_time;
    start = clock();

    //-- Step 1: Detect keypoints

    SurfFeatureDetector detector(minHessian);

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img1, keypoints_1 );
    detector.detect( img3, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    extractor.compute( img1, keypoints_1, descriptors_1 );
    extractor.compute( img3, keypoints_2, descriptors_2 );

    //-- Step 3: Choose only the "good" matches
    featureMatch detect;
    good_matches = detect.goodmatchDetect(keypoints_1, keypoints_2, img1, img3);

    feat_time = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout<<"Time for feature matching: "<< feat_time << endl;

    waitKey(0);


    //-----------------------------------------------CONVERT TWO IMAGES TO DISPARITY IMAGE-----------------------------------------------

    StereoSGBM sgbm(minDisparity, numberOfDisparities, SADWindowSize, P1, P2, disp12MaxDiff, preFilterCap,\
        uniquenessRatio, speckleWindowSize, speckleRange, fullDP);  

    int64 t = getTickCount();

    // convert the two images into a disparity image
    sgbm(img1, img2, disp);
    t = getTickCount() - t;
    printf("SGBM time elapsed: %fms\n", t*1000/getTickFrequency());

    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else 
        disp.convertTo(disp8, CV_8U);

    //---------------------------------------------------WRITE 3D CLOUD TO FILE--------------------------------------------    
    
    // Project to 3D, points filled into xyz mat format
    reprojectImageTo3D(disp8, xyz, Q, true);

    //-------------------------------------------------FIND FEATURE POINTS IN 3D----------------------------------------------

    //This loops through all of the matched feature points and finds the image coordinate in 3D. 
    //If the z-coordinate is less than 3D (ie if the block matching found a match),
    //then the 3d points are saved to a vector and the 2D feature points in the second image
    for( int i = 0; i < (int)good_matches.size(); i++ )
    {
        u = keypoints_1[good_matches[i].queryIdx].pt.x;
        v = keypoints_1[good_matches[i].queryIdx].pt.y;
        Vec3f point = xyz.at<Vec3f>(v, u);
        if (point[2]<10000)
        {
            list_points3d.push_back(Point3f(point[0],point[1],point[2]));
            list_points2d.push_back(Point2f(keypoints_2[good_matches[i].trainIdx].pt.x, keypoints_2[good_matches[i].trainIdx].pt.y));
            keypoints_im2.push_back(KeyPoint(keypoints_2[good_matches[i].trainIdx]));
        }
    }

//----------------------------------------------------------SOLVE PNP------------------------------------------------------

    int64 t_pnp = getTickCount();

    solvePnPRansac( list_points3d, list_points2d, M1, distCoeffs, rvec, tvec, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers, flags);
    
    Rodrigues(rvec,R_matrix);
    t_matrix = tvec;

    cout << "Rotation matrix: " << R_matrix << endl;
    cout << "Translation matrix: " << t_matrix << endl;
    cout << "\n"<<"Inlier indices: " << inliers << endl;

    // end clock, for kicks
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;

    std::cout<< "\n" <<"Total time: "<< duration <<'\n';


    return 0;
}