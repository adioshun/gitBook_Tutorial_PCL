# [Identifying ground returns using ProgressiveMorphologicalFilter segmentation](http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering)

> 항공 Lidar 


```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

// Identifying ground returns using ProgressiveMorphologicalFilter segmentation
// http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering

int
main (int argc, char** argv)
{  
  // *.PCD 파일 읽기
  // https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/sample/samp11-utm.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);
  
  // 포인트 정보 출력
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 오브젝트 생성 
  pcl::PointIndicesPtr ground (new pcl::PointIndices);
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // 오브젝트 생성 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);  
  //extract.setNegative (true);  //samp11-utm_object.pcd 
  extract.filter (*cloud_filtered);

  // 포인트 정보 출력
  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // 생성된 포인트클라우드(inlier) 저장 
  pcl::io::savePCDFile<pcl::PointXYZ>("samp11-utm_ground.pcd", *cloud_filtered);
  
  return (0);
}
```
