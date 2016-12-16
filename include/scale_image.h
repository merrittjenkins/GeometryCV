#ifndef SCALE_IMAGE_H
#define SCALE_IMAGE_H

#include "standard_includes.h"

using namespace cv;

class scaleImage {
	Mat img_big; 
	Mat img_med;
    Mat img_small;


  public:

  	//scaleImage(std::string img_filename, int colormode);
  	Mat downSize(std::string img_filename, int colormode);

};

#endif