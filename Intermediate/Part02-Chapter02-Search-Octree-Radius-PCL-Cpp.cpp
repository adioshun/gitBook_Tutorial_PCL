#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_cluster_0.pcd", *cloud);

  float resolution = 128.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  pcl::PointXYZRGB searchPoint;
  searchPoint.x = 0.026256f;
  searchPoint.y = -1.464739f;
  searchPoint.z = 0.929567f;

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with radius=" << radius << std::endl;


  if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


}