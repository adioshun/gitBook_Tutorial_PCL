# Region-Growing-PCL-Cpp \(50%\)

```cpp
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

// Region growing segmentation
// http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation
// Commnets : Hunjung, Lim (hunjung.lim@hotmail.com)

int
main (int argc, char** argv)
{
  // *.PCD 파일 읽기 
  // https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/sample/RANSAC_plane_true.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile <pcl::PointXYZ> ("RANSAC_plane_true.pcd", *cloud);

  // 알고리즘에서 사용하는 Surface Normal 계산 
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  //pcl::IndicesPtr indices (new std::vector <int>);
  //pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFile<pcl::PointXYZRGB>("result_region_growing_segmentation.pcd", *colored_cloud);


  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud(colored_cloud);
  //while (!viewer.wasStopped ())
  //{
  //}

  return (0);
}
```

