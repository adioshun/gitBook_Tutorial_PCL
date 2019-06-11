#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;


     seg.setInputCloud (cloud);
     seg.segment (*inliers, *coefficients);

     // Extract the inliers
     extract.setInputCloud (cloud);
     extract.setIndices (inliers);
     extract.setNegative (false);//true
     extract.filter (*cloud_p);
     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

     std::stringstream ss;
     ss << "RANSAC_plane.pcd";
     pcl::PCDWriter writer2;
     writer2.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

 

  return (0);
}