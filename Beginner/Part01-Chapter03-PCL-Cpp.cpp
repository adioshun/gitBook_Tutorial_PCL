#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);

  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.70, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;
  
  
  

  pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_passthrough.pcd", *cloud_filtered); //Default binary mode save

  return (0);
}