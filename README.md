# GeometryCV
Point Cloud Stitching

These files, when run in sequence, will load stereo image pairs, generate point clouds, filter the point clouds, and stitch them together

Step 1:
	Run StereoMatching.py by typing:
	$ python StereoMatching.py

	StereoMatching.py will read from the CSV file StalkLocations.csv which contains the x,z offset of each image. The python script also loads the stereo pairs contained in the folder /images and 	outputs point cloud into a folder /plyClouds2. The python script iterates through image pairs using a for-loop. To load more or fewer images, adjust the number of iterations in the for-loop.


Step 2:
	Build the C++ files using make
	From the build folder, run statOutlierBundled.cpp by typing:
	$ ./statOutlierBundled ../plyClouds2/cloud{0..12}.ply
	This will send 13 point clouds into statOutlierBundled.cpp
