#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

int
  main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud_b;
  pcl::PLYReader reader;
  
  for (int i = 1; i < argc; i++)
  {
  	  reader.read<pcl::PointXYZRGB> (argv[i], *cloud_a);

	  std::cerr << "Cloud before concatenation: " << std::endl;
	  std::cerr << *cloud_a << std::endl;

  	  // Create the filtering object
  	  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  	  sor.setInputCloud (cloud_a);
  	  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	  sor.filter (*cloud_a_filtered);

  	  std::cerr << "PointCloud after filtering: " << cloud_a_filtered->width * cloud_a_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_a_filtered) << ").";

	  cloud_b+=*cloud_a_filtered;
  }  

  std::cerr << cloud_b << std::endl;

  pcl::PLYWriter writer;

  writer.write<pcl::PointXYZRGB> ("../plyClouds2/Concatenate_BIG.ply", cloud_b, false);	

  return (0);
}

