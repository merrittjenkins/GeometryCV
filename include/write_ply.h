#ifndef WRITE_PLY_H
#define WRITE_PLY_H

#include "standard_includes.h"

using namespace cv;

class plyWrite {

	double max_z;
	FILE* fp;
	string ply;
	string format;
	string vertex;
	string x; 
	string y; 
	string z;

  public:

	plyWrite(const char* filename, const Mat& mat);
	void saveXYZRGB(const Mat& mat, const Mat& img1);//const char* filename, const Mat& mat, const Mat& img1);
	void saveXYZ(const Mat& mat, const Mat& img1);//const char* filename, const Mat& mat, const Mat& img1);
};

#endif