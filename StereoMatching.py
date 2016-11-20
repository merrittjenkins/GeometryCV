
'''
Simple example of stereo image matching and point cloud generation.
Resulting .ply file cam be easily viewed using MeshLab ( http://meshlab.sourceforge.net/ )
'''

import numpy as np
import cv2
import csv
import itertools

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'w') as f:
        f.write(ply_header % dict(vert_num=len(verts)))
        np.savetxt(f, verts, '%f %f %f %d %d %d')
        f.close()


if __name__ == '__main__':

	# Load the csv file and count the number of rows. I need the number of rows to initilize the variable "offset"
    ifile  = open('StalkLocations.csv', "rb")
    reader = csv.reader(ifile)

    row_count = sum(1 for spot in reader)
    offset = np.zeros((row_count, 2))
    ifile.close()

	#This is a shitty hack. I'm closing the file and reopening it to iterate through the files
    ifile  = open('StalkLocations.csv', "rb")
    reader = csv.reader(ifile)
    rownum = 0
    for row in reader:
        if rownum == 0:
            header = row
        else:
            colnum = 0
            for col in row:
                #print '%-8s: %s' % (header[colnum], col)
                offset[rownum][colnum] = col
                colnum += 1                		        
        rownum += 1
    ifile.close()

	# Loop through the images
    for i in range(0,20,4):

		doubledigit = ('{num:02d}'.format(num=i))

		# Load the images - the first image will be 00, then 01, etc
		imgL = cv2.pyrDown( cv2.imread('images/left0000{0}.jpg'.format(doubledigit)))  # downscale images for faster processing
		imgR = cv2.pyrDown( cv2.imread('images/right0000{0}.jpg'.format(doubledigit)))

		# Disparity range is tuned only marginally. I'm sure that it can be improved.
		window_size = 10
		min_disp = 100
		num_disp = 496 #208 #528
		stereo = cv2.StereoSGBM(minDisparity = min_disp,
		    numDisparities = num_disp,
		    SADWindowSize = window_size,
		    uniquenessRatio = 10, #10
		    speckleWindowSize = 100, #100
		    speckleRange = 10, #32
		    disp12MaxDiff = 10,
		    P1 = 8*3*window_size**2,
		    P2 = 32*3*window_size**2,
		    fullDP = False
		)

		print 'computing disparity #2...'
		disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

		print 'generating 3d point cloud #2...',
		h, w = imgL.shape[:2]
		f = 0.8*w                          # guess for focal length

		Q = np.float32([[1, 0, 0, -0.5*w],
		                [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
		                [0, 0, 0,     -f], # so that y-axis looks up
		                [0, 0, 1,      0]])
		points2 = cv2.reprojectImageTo3D(disp, Q)
		colors2 = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
		mask = disp > disp.min()

		# create a vector representing the x_offset 
		#x_adder = i*-.7*np.ones(len(points2[mask]))
		x_adder = offset[i+1][0]*np.ones(len(points2[mask]))
		
		#convert this vector to be the right dimension
		x_adder = x_adder[:, None]

		#create array of zeros
		zero_add = np.zeros(len(points2[mask]))
		zero_add = zero_add[:, None] 

		# create a vector representing the x_offset 
		#z_adder = i*-.15*np.ones(len(points2[mask]))
		z_adder = offset[i+1][1]*np.ones(len(points2[mask]))	
		#convert this vector to be the right dimension
		z_adder = z_adder[:, None]

		# I couldn't figure out how to append three arrays, so I append two, and then do it again
		full_addition = np.append(z_adder, x_adder, axis=1)
		full_addition = np.append(full_addition, zero_add, axis=1)

		# Add an array of z_offsets, x_offsets, and zeros to the points
		out_points2 = np.add(full_addition, points2[mask])

		#comment out if creating an appended cloud
		out_colors2 = colors2[mask]

		out_fn = 'cloud{0}.ply'.format(doubledigit) 
		write_ply('plyClouds2/cloud{0}.ply'.format(doubledigit), out_points2, out_colors2)
		print '%s saved' % 'cloud{0}.ply'.format(doubledigit)
	

