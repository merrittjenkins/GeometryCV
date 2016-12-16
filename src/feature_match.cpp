#include "standard_includes.h"
#include "feature_match.h"

using namespace cv;
using namespace std;

std::vector<DMatch> featureMatch::goodmatchDetect(std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, Mat img1, Mat img3){

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
    { if( matches[i].distance <= max(4*min_dist, 0.02) )  //these numbers affect the number of keypoint pairs chosen
        { good_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img1, keypoints_1, img3, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    //-- Show detected matches
    namedWindow("Good Matches",WINDOW_NORMAL);
    resizeWindow("Good Matches", 1600,800);
    imshow( "Good Matches", img_matches );


    //cout << "Number of inlier matches: "  << good_matches.size() << endl;

    //cout << "U,V COORDS OF FIRST MATCHED KEYPOINT: " << keypoints_1[good_matches[0].queryIdx].pt << endl;//good_matches[1].distance << endl;

    return(good_matches);
}