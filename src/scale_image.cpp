#include "standard_includes.h"
#include "scale_image.h"

using namespace cv;
using namespace std;

/*
scaleImage::scaleImage(string img_filename, int color_mode){
    img_big = imread(img_filename, color_mode);

    cout << "Original Size:" << endl;
    cout << img_big.rows << endl;
    cout << img_big.cols << "\n" << endl;
}
*/

Mat scaleImage::downSize(string img_filename, int color_mode){

    img_big = imread(img_filename, color_mode);

    pyrDown(img_big, img_med, Size(img_big.cols/2, img_big.rows/2));

    pyrDown(img_med, img_small, Size(img_med.cols/2, img_med.rows/2));

    return(img_small);

}