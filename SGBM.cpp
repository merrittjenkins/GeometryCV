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
    std::string img1_filename = "left000007.jpg";
    std::string img2_filename = "right000007.jpg";
    std::string img3_filename = "left000006.jpg";
    std::string disparity_filename = "disparity.jpg";

    const char* point_cloud_filename = 0;

    point_cloud_filename = "cloud.ply";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;

    int color_mode = alg == STEREO_BM ? 0 : -1;

    Mat img1_big = imread(img1_filename, color_mode);
    Mat img2_big = imread(img2_filename, color_mode);
    Mat img3_big = imread(img3_filename, color_mode);

    cout << img1_big.rows << endl;
    cout << img1_big.cols << endl;

    Mat img1; Mat img2; Mat img3;
    //pyrDown(img1_big, img1, Size(img1_big.cols/2, img1_big.rows/2));
    //pyrDown(img2_big, img2, Size(img2_big.cols/2, img2_big.rows/2));

    img1=img1_big;
    img2=img2_big;
    img3=img3_big;

    Size img_size = img1.size();


    //------------------------------------FIND MATCHES-----------------------------------------
        clock_t start;
    double duration;
    start = clock();

    int minHessian = 800;
    SurfFeatureDetector detector(minHessian);

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img1, keypoints_1 );
    detector.detect( img3, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;

    extractor.compute( img1, keypoints_1, descriptors_1 );
    extractor.compute( img3, keypoints_2, descriptors_2 );

    cout << "Im here 1!!" << endl;

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    cout << "Im here 2!!" << endl;

    //Orb features need converted file type
    if(descriptors_1.type()!=CV_32F) {
        descriptors_1.convertTo(descriptors_1, CV_32F);
        descriptors_2.convertTo(descriptors_2, CV_32F);
    }

    cout << "Im here 3!!" << endl;
    matcher.match( descriptors_1, descriptors_2, matches );

    //cout << "herrooo" << matches << endl;

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
    { if( matches[i].distance <= max(2*min_dist, 0.5) )  //this used to be 0.02
        { good_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img1, keypoints_1, img3, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout<<"printf: "<< duration <<'\n';

    //-- Show detected matches
    namedWindow("Good Matches",WINDOW_NORMAL);
    resizeWindow("Good Matches", 1600,800);
    imshow( "Good Matches", img_matches );

    for( int i = 0; i < (int)good_matches.size(); i++ )
    { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

    cout << "U,V COORDS OF FIRST MATCHED KEYPOINT: " << keypoints_1[good_matches[0].queryIdx].pt << endl;//good_matches[1].distance << endl;

    Mat matchedKeypoints[2][good_matches.size()];

    //for( int i = 0; i < (int)good_matches.size(); i++ )
    //{
    //    cout << keypoints_1[good_matches[i].queryIdx].pt << endl;
    //}


    waitKey(0);

    //-----------------------------------------------PROJECT OUT TO 3D-----------------------------------------------
    

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

    sgbm(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);


        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        printf("press any key to continue...");
        fflush(stdout);
        waitKey();
        printf("\n");

    if(!disparity_filename.empty())
        printf("writing the disparity file...");
        imwrite(disparity_filename, disp8);

    //Mat mask = disp8 > 79;
    //cout << mask.rows << endl;
    //cout << mask.cols << endl;
    //imshow( "MASK", mask);
    //cout << mask << endl;
    //Mat outcolors = img1(mask);

    Mat M1, D1, M2, D2;
    M1 = (Mat_<double>(3,3) << 2421.988247695817, 0.0, 1689.668741757609, 0.0, 2424.953969600827, 1372.029058638022, 0.0, 0.0, 1.0);
    M2 = (Mat_<double>(3,3) << 2402.822932836092, 0.0, 1715.977908625392, 0.0, 2405.531742202912, 1356.326619144245, 0.0, 0.0, 1.0);
    D1 = (Mat_<double>(5,1) << -0.001013934596658032, 0.00355911874186913, -0.0006557984914098192, -0.0009806454600919795, 0.0);
    D2 = (Mat_<double>(5,1) << -0.00596545966775133, 0.0056628943340289, -0.0008602296090100512, 6.54885403584621e-05, 0.0);

    //[2000,0,0; 0, 2000,0; 0, 0, 1] >> M1;

    Mat R, T, R1, P1_rec, R2, P2_rec;
    Mat Q;
    R = (Mat_<double>(3,3) << 0.9999698296761699, 0.000573862809423943, -0.00774663919954301, -0.000574843594088605, 0.9999998270419937, -0.0001243817708070411, 0.007746566481627299, 0.0001288311240883246, 0.9999699866047417); //1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    T = (Mat_<double>(3,1) << 0.0, 2.0, 0.0); //-219.5597042086226, 0.0, 0.0);

    
    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1_rec, P2_rec, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );  

    cout << Q << endl;  

    //if(point_cloud_filename)
    //{
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        //cvtColor(img1, COLOR_BGR2RGB)

        saveXYZ(point_cloud_filename, xyz, img1);
        printf("\n");
    //}

//-------------------------------------------------PROJECT 3D POINTS TO IMAGE 6----------------------------------------------





    return 0;
}