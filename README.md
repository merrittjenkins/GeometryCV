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
	$ ./statOutlierBundled ../plyClouds2/cloud{00..12}.ply
	This will send 13 point clouds into statOutlierBundled.cpp

Step 3: 
	Run incrementalRegister.cpp by typing:
	$ ./incrementalRegister  Cloud{0..12}_inliers.pcd
	The 13 files CloudXX_inliers.pcd should be automatically located in the Build folder after running statOutlierBundled.cpp
	When running incrementalRegister.cpp, you may notice a delay of 30-40s. This is because the script is loading all of the pcd files at once.
	Follow the prompt, pressing 'q' when appropriate.
	This script will output X-1 files (i.e. outputs 12 files if you input 13 files). This is because each new cloud consists of a stitched cloud pair.
	The new stitched clouds will be stored in the plyClouds2 folder under the names cloudX_registered.ply.
	You can load all of the clouds into Meshlab at once for viewing.
