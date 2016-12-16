#ifndef FEATURE_MATCH_H
#define FEATURE_MATCH_H

#include "standard_includes.h"

using namespace cv;

class featureMatch {

  public:
  	
	std::vector<DMatch> goodmatchDetect(std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, Mat img1, Mat img3);
};

#endif