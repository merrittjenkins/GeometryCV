#include "standard_includes.h"
#include "write_ply.h"

using namespace cv;
using namespace std;


 plyWrite::plyWrite(const char* filename, const Mat& mat)
 {
    max_z = 1.0e4;
    fp = fopen(filename, "wt");
    std::ostringstream stream;
    int i=0;

    cout << "mat.rows: " << mat.rows << endl;
    cout << "mat.cols: " << mat.cols << endl;

    for(int row = 0; row < mat.rows; row++)
    {
        for(int col = 0; col < mat.cols; col++)
        {
            Vec3f point = mat.at<Vec3f>(row,col);
            //if (row<5 && col<5){cout << point[0] << ", " << point[1] << ", " << point[2] << endl;}

            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            i++;
            //cout << i << endl;
        }
    }

    cout << "OUT OF HERE!" << endl;

    ply = "ply";
    format = "format ascii 1.0";
    cout << format << endl;
    //string vertex = ("element vertex %i", mat.rows);
    //string vertex = "element vertex " + mat.rows;
    stream << "element vertex " << i;
    vertex = stream.str();
    cout << "Number of vertices:" << i << endl;
    x = "property float x";
    y = "property float y";
    z = "property float z";
 }

void plyWrite::saveXYZRGB(const Mat& mat, const Mat& img1)//const char* filename, const Mat& mat, const Mat& img1)
{
    int counter = 0;
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

    cout << "mat.rows: " << mat.rows << endl;
    cout << "mat.cols: " << mat.cols << endl;

    for(int row = 0; row < mat.rows; row++)
    {
        for(int col = 0; col < mat.cols; col++)
        {
            Vec3f point = mat.at<Vec3f>(row,col);
            Vec3b intensity = img1.at<Vec3b>(row,col);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            //if (row<5 && col<5){cout << point[0] << ", " << point[1] << ", " << point[2] << endl;}

            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f %i %i %i\n", point[0], point[1], point[2], red, green, blue);
            counter++;

        }
    }
    cout << "COUNTER IS: " << counter << endl;
    fclose(fp);
}

void plyWrite::saveXYZ(const Mat& mat, const Mat& img1)
{

    int counter = 0;
    string header = "end_header";
    fprintf(fp, "%s\n", ply.c_str());
    fprintf(fp, "%s\n", format.c_str());
    fprintf(fp, "%s\n", vertex.c_str());
    fprintf(fp, "%s\n", x.c_str());
    fprintf(fp, "%s\n", y.c_str());
    fprintf(fp, "%s\n", z.c_str());
    fprintf(fp, "%s\n", header.c_str());

    cout << "mat.rows: " << mat.rows << endl;
    cout << "mat.cols: " << mat.cols << endl;

    for(int row = 0; row < mat.rows; row++)
    {
        for(int col = 0; col < mat.cols; col++)
        {
            Vec3f point = mat.at<Vec3f>(row,col);
            Vec3b intensity = img1.at<Vec3b>(row,col);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            //if (row<5 && col<5){cout << point[0] << ", " << point[1] << ", " << point[2] << endl;}

            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f %i %i %i\n", point[0], point[1], point[2], red, green, blue);
            counter++;

        }
    }
    cout << "COUNTER IS: " << counter << endl;
    fclose(fp);

}