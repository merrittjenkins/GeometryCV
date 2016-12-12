/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

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

using namespace cv;
using namespace std;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [--no-display]\n");
}

static void saveXYZ(const char* filename, const Mat& mat, const Mat& img1)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    std::ostringstream stream;
    int i=0;

    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                i++;
        }
    }

    string ply = "ply";
    string format = "format ascii 1.0";
    cout << format << endl;
    //string vertex = ("element vertex %i", mat.rows);
    //string vertex = "element vertex " + mat.rows;
    stream << "element vertex " << i;
    string vertex = stream.str();
    cout << vertex << endl;
    string x = "property float x";
    string y = "property float y";
    string z = "property float z";
    string r = "property uchar red";
    string g = "property uchar green";
    string b = "property uchar blue";
    string header = "end_header";
    fprintf(fp, "%s\n", ply.c_str());
    fprintf(fp, "%s\n", format.c_str());
    fprintf(fp, "%s\n", vertex.c_str());
    fprintf(fp, "%s\n", x.c_str());
    fprintf(fp, "%s\n", y.c_str());
    fprintf(fp, "%s\n", z.c_str());
    fprintf(fp, "%s\n", r.c_str());
    fprintf(fp, "%s\n", g.c_str());
    fprintf(fp, "%s\n", b.c_str());
    fprintf(fp, "%s\n", header.c_str());

    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            Vec3b intensity = img1.at<Vec3b>(y, x);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f %i %i %i\n", point[0], point[1], point[2], red, green, blue);
        }
    }
    fclose(fp);
}

int main(int argc, char** argv)
{

    //------------------------------------LOAD IMAGES-----------------------------------------

    //The first two images are the stereo pair. The third image is the one that has moved (we don't know its position)
    std::string img1_filename = "left000006.jpg";
    std::string img2_filename = "right000006.jpg";
    std::string img3_filename = "left000005.jpg";
    std::string disparity_filename = "disparity.jpg";

    //cloud.ply is the point cloud of the plant, features.ply are the 2D features out in 3D
    const char* point_cloud_filename = 0;
    const char* point_cloud_filename_2 = 0;

    point_cloud_filename = "cloud.ply";
    point_cloud_filename_2 = "features.ply";

    //assign SGBM because it gives superior results
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;

    int color_mode = alg == STEREO_BM ? 0 : -1;

    Mat img1_big = imread(img1_filename, color_mode);
    Mat img2_big = imread(img2_filename, color_mode);
    Mat img3_big = imread(img3_filename, color_mode);

    cout << img1_big.rows << endl;
    cout << img1_big.cols << endl;

    //I was originally downsampling but changed my mind. Leaving the code here in case I change it back.
    Mat img1; Mat img2; Mat img3;
    //pyrDown(img1_big, img1, Size(img1_big.cols/2, img1_big.rows/2));
    //pyrDown(img2_big, img2, Size(img2_big.cols/2, img2_big.rows/2));

    img1=img1_big;
    img2=img2_big;
    img3=img3_big;

    Size img_size = img1.size();


    //------------------------------------FIND MATCHES-----------------------------------------
    
    //start a timer just for kicks
    clock_t start;
    double duration;
    start = clock();

    //-- Step 1: Detect keypoints
    int minHessian = 800; //the hessian appears to affect the number of keypoints. I don't understand what it does.
    SurfFeatureDetector detector(minHessian);

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img1, keypoints_1 );
    detector.detect( img3, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;

    extractor.compute( img1, keypoints_1, descriptors_1 );
    extractor.compute( img3, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;

    //Orb features need converted file type
    if(descriptors_1.type()!=CV_32F) {
        descriptors_1.convertTo(descriptors_1, CV_32F);
        descriptors_2.convertTo(descriptors_2, CV_32F);
    }

    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    int i;
    for(i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    cout << "Number of matches: " << i << endl;

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )  //these numbers affect the number of keypoint pairs chosen
        { good_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img1, keypoints_1, img3, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // end clock, for kicks
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout<<"Timer: "<< duration <<'\n';

    //-- Show detected matches
    namedWindow("Good Matches",WINDOW_NORMAL);
    resizeWindow("Good Matches", 1600,800);
    imshow( "Good Matches", img_matches );

    cout << "Number of inlier matches: "  << good_matches.size() << endl;

    //for( int i = 0; i < (int)good_matches.size(); i++ )
    //{ //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
    
    //}

    cout << "U,V COORDS OF FIRST MATCHED KEYPOINT: " << keypoints_1[good_matches[0].queryIdx].pt << endl;//good_matches[1].distance << endl;

    waitKey(0);

    //-----------------------------------------------CONVERT TWO IMAGES TO DISPARITY IMAGE-----------------------------------------------

    int SADWindowSize;
    bool no_display;
    float scale;

    int window_size = 11;
    int min_disp = 100;
    //int num_disp = 320;
    int num_disp = 800;

    int minDisparity;
    int numberOfDisparities;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int P1;
    int P2;
    bool fullDP;
    int preFilterCap;
    int mode;

    //Ptr<StereoBM> bm = StereoBM::create(496,11);
    //StereoSGBM sgbm();

    StereoSGBM sgbm(minDisparity = min_disp,
		    numberOfDisparities = num_disp,
		    SADWindowSize = window_size,
		    P1 = pow(8*3*window_size,2),
		    P2 = pow(32*3*window_size,2),
 	            disp12MaxDiff = 10,
                    preFilterCap = 0,
		    uniquenessRatio = 10,
		    speckleWindowSize = 200,
		    speckleRange = 1,
		    fullDP=false);  


    Rect roi1, roi2;

    Mat disp, disp8;

    int64 t = getTickCount();

    // convert the two images into a disparity image
    sgbm(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    if( alg != STEREO_VAR )
        {
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        //disp.convertTo(disp8, CV_8U, 1/16.);
        cout << "Alg is not STEREO_VAR" << endl;
        //disp.convertTo(disp8, CV_8U);
        }
    else {
        disp.convertTo(disp8, CV_8U);}

        //cout << disp8 << endl;

        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        printf("press any key to continue...");
        fflush(stdout);
        waitKey();
        printf("\n");

    if(!disparity_filename.empty())
        printf("writing the disparity file...");
        imwrite(disparity_filename, disp8);

    // M1/M2 are camera intrinsics (known) and D1/D2 are camera distortion parameters (known)
    Mat M1, D1, M2, D2;
    M1 = (Mat_<double>(3,3) << 2421.988247695817, 0.0, 1689.668741757609, 0.0, 2424.953969600827, 1372.029058638022, 0.0, 0.0, 1.0);
    M2 = (Mat_<double>(3,3) << 2402.822932836092, 0.0, 1715.977908625392, 0.0, 2405.531742202912, 1356.326619144245, 0.0, 0.0, 1.0);
    D1 = (Mat_<double>(5,1) << -0.001013934596658032, 0.00355911874186913, -0.0006557984914098192, -0.0009806454600919795, 0.0);
    D2 = (Mat_<double>(5,1) << -0.00596545966775133, 0.0056628943340289, -0.0008602296090100512, 6.54885403584621e-05, 0.0);

    // R is the rotation between the two lenses (pretty much identity)
    // T is the translation between the two lenses (~10cm, but that gives weird world units...)
    //Mat R, T, R1, P1_rec, R2, P2_rec;
    Mat Q;
    //R = (Mat_<double>(3,3) << 0.9999698296761699, 0.000573862809423943, -0.00774663919954301, -0.000574843594088605, 0.9999998270419937, -0.0001243817708070411, 0.007746566481627299, 0.0001288311240883246, 0.9999699866047417); //1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    //R = (Mat_<double>(3,3) << 1.0220, 0, 0.0046, 0, 1.0208, 0.0021, 0, 0, 1); //1,0,0,0,1,0,0,0,1);
    //T = (Mat_<double>(3,1) << -.0914, 0.0, 0.0); //-219.5597042086226, 0.0, 0.0);

    //This is just to get a Q matrix
    //stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1_rec, P2_rec, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );   

    float cx = 1688;
    float cy = 1352;
    float f = 2421;
    float T = -.0914; 

    Q = (Mat_<double>(4,4) << -1,0,0,cx,0,1,0,-cy,0,0,0,f,0,0,1/T,0);
    cout << "\n" << Q << endl;

    //---------------------------------------------------WRITE 3D CLOUD TO FILE--------------------------------------------    

    printf("storing the point cloud...");
    fflush(stdout);
    Mat xyz;
    reprojectImageTo3D(disp8, xyz, Q, true);
    //cvtColor(img1, COLOR_BGR2RGB)

    saveXYZ(point_cloud_filename, xyz, img1);
    printf("\n");


    //-------------------------------------------------FIND FEATURE POINTS IN 3D----------------------------------------------

    FILE* fp = fopen(point_cloud_filename_2, "wt");
    std::ostringstream stream;

    float u;
    float v;

    vector<Point3f> list_points3d;
    vector<Point2f> list_points2d;
    vector<KeyPoint> keypoints_im2;

    //This loops through all of the matched feature points and finds the image coordinate in 3D. If the z-coordinate is less than 3D (ie if the block matching found a match),
    //then the 3d points are saved to a vector and the 2D feature points in the second image (image 6)
    for( int i = 0; i < (int)good_matches.size(); i++ )
    {
        //cout << keypoints_1[good_matches[i].queryIdx].pt << endl;
        u = keypoints_1[good_matches[i].queryIdx].pt.x;
        v = keypoints_1[good_matches[i].queryIdx].pt.y;
        Vec3f point = xyz.at<Vec3f>(v, u);
        if (point[2]<10000)
        {
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);  //This prints the feature points to another file
            list_points3d.push_back(Point3f(point[0],point[1],point[2]));
            list_points2d.push_back(Point2f(keypoints_2[good_matches[i].trainIdx].pt.x, keypoints_2[good_matches[i].trainIdx].pt.y));
            keypoints_im2.push_back(KeyPoint(keypoints_2[good_matches[i].trainIdx]));
        }
    }
    fclose(fp);

//----------------------------------------------------------SOLVE PNP------------------------------------------------------

    bool flags = 1;
    Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);
    Mat rvec = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);

    Mat R_matrix = Mat::zeros(3, 3, CV_64FC1);
    Mat t_matrix = Mat::zeros(3, 1, CV_64FC1);

    bool useExtrinsicGuess = false;
    int iterationsCount=2000;
    float reprojectionError=50.0;
    double confidence=0.99;
    //InputOutputArray inliers=noArray();
    Mat inliers;

    solvePnP( list_points3d, list_points2d, M1, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

    //solvePnPRansac( list_points3d, list_points2d, M1, distCoeffs, rvec, tvec, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers, flags);
    
    Rodrigues(rvec,R_matrix);
    t_matrix = tvec;

    cout << "R_matrix: " << R_matrix << endl;
    cout << "t_matrix: " << t_matrix << endl;
    cout << "inliers: " << inliers << endl;


    Mat jacobian;
    double aspectRatio = 0;
    vector<Point2f> imagePoints; 
    projectPoints (list_points3d,rvec,tvec,M1,distCoeffs,imagePoints,jacobian,aspectRatio);   

    cout << imagePoints << endl;
    cout << "Number of imagePoints: " << imagePoints.size() << endl;

    Point pt;
    vector<KeyPoint> keypoints_projected;
    for( size_t i = 0; i < imagePoints.size(); i++ ) {
        keypoints_projected.push_back(KeyPoint(imagePoints[i].x,imagePoints[i].y, 1.f));
        //cout << "imagePoints" << imagePoints[i].x << endl;
        //pt =  Point(imagePoints[i].x,imagePoints[i].y);
        cout << "Keypoint: " << keypoints_projected[i].pt << endl;
    }


    Mat im_keypoints;
    drawKeypoints(img3, keypoints_im2, im_keypoints, Scalar::all(-1),4);

    Mat im_comparekeypoints;
    int radius = 50;
    int thickness=20;
    int lineType=8;
    int shift=0;
    drawKeypoints(img3, keypoints_projected, im_comparekeypoints, Scalar::all(-1),0);
    //circle(img3, imagePoints, radius, Scalar::all(-1), thickness, lineType, shift);

    namedWindow("Matches",WINDOW_NORMAL);
    resizeWindow("Matches", 800,800);
    imshow("Matches", im_keypoints);

    cout << "Here" << endl;

    namedWindow("Matches2",WINDOW_NORMAL);
    resizeWindow("Matches2", 800,800);
    imshow("Matches2", im_comparekeypoints);

    waitKey(0);

    return 0;
}