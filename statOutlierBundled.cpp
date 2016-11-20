#include <stdio.h>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


using namespace std;
using namespace pcl;

//TO RUN: Call this script with ./statOutlierBundled ../plyClouds2/cloud{0..12].ply
//TO-DO: 
//       -add passthrough filter
//       -remove the neighbor filter or reduce its size 

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PLYReader reader;
  //pcl::PCDReader reader;

  // Print number of arguments sent in
  cout << "argc: " << argc << endl;

  // Parse the first argument to know the starting point cloud number
  std::string s = argv[1];
  std::string delimiter = "cloud";

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    //std::cout << token << std::endl;
    s.erase(0, pos + delimiter.length());
  }
  std::cout << s << std::endl;
  std::string delimiter2 = ".";
  std::string cloudNumber = s.substr(0, s.find(delimiter2));

  // Print the starting cloud number
  cout << cloudNumber << endl;

  // Convert that cloud number to an int
  std::string myString = "45";
  int value = atoi(cloudNumber.c_str());
  cout << value << endl;

  for (int i = 1; i < argc; i++)
	{
	  cout << argv[i] << endl;

	  //reader.read<pcl::PointXYZRGB> ("../plyClouds/cloud2.ply", *cloud);
	  reader.read<pcl::PointXYZRGB> (argv[i], *cloud);

	  std::cerr << "Cloud before filtering: " << std::endl;
	  std::cerr << *cloud << std::endl;


	  //---------Create a conditional removal filter to remove black pixels---------
	  int rMax = 255;
	  int rMin = 15;
	  int gMax = 255;
	  int gMin = 15;
	  int bMax = 255;
	  int bMin = 15;
	  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
	  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

	  // build the filter
	  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
	  condrem.setInputCloud (cloud);
	  condrem.setKeepOrganized(true);
	  // apply filter
	  condrem.filter (*cloud_filtered); 


	  //-----Create the statistical outlier filtering object-----
	  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	  // set thresholds,tree size, and apply filter
	  sor.setInputCloud (cloud_filtered);
	  sor.setMeanK (50);
	  sor.setStddevMulThresh (1.0);
	  sor.filter (*cloud_filtered);
	  // print post-filtered size
	  std::cerr << "Cloud after filtering: " << std::endl;
	  std::cerr << *cloud_filtered << std::endl;
	  // remove the nans so we can perform more filtering later
	  std::vector<int> indices;
	  pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, indices);


	  //-----Create the radius outlier removal filtering object-----
	  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	  // build the filter
	  outrem.setInputCloud(cloud_filtered);
	  outrem.setRadiusSearch(0.1);
	  outrem.setMinNeighborsInRadius (500);
	  // apply filter
	  outrem.filter (*cloud_filtered);


	  //-----Write to a .pcd file in the build folder-----

	  pcl::PCDWriter writer;

	  std::string result;
	  std::stringstream sstm;
	  sstm << "Cloud" << i-1+value << "_inliers" << ".pcd";
	  result = sstm.str();

	  writer.write<pcl::PointXYZRGB> (result, *cloud_filtered, false);	
	}

  return (0);
}
