README

To compile, you need OpenCV 2.4 and the nonfree packages. To install the nonfree packages, see here: 

http://stackoverflow.com/questions/27481849/include-nonfree-opencv-2-4-10-on-ubuntu

Compile by typing:
$ cmake .
$ make

To run the script SGBM.cpp, run like this: ./SGBM

You will see an image pop up, and the script will stop running. You must press the Enter key while the image is open in order to continue running.

The script writes to a file cloud.ply. This is the sorghum point cloud and can be viewed in Meshlab.
The script also writes to a file called features.ply. This file is not actually a ply file because it has no header. The file consists of the 3D locations of the feature points, and one must manually add a header to view the file (most likely not necessary for anything you will do). 
