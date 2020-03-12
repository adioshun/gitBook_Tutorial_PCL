# [Color-based region growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php#region-growing-rgb-segmentation)


There are two main differences in the color-based algorithm. 

The first one is that it uses **color** instead of normals. 

The second is that it uses the **merging algorithm** for over- and under- segmentation control. 

Let’s take a look at how it is done. 
- After the segmentation, an attempt for merging clusters with close colors is made. 
- Two neighbouring clusters with a small difference between average color are merged together. 
- Then the second merging step takes place. 
- During this step every single cluster is verified by the number of points that it contains. 
- If this number is less than the user-defined value than current cluster is merged with the closest neighbouring cluster.

```cpp
#include <iostream>
#include <thread>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

// Color-based region growing segmentation
// http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php#region-growing-rgb-segmentation

int
main (int argc, char** argv)
{
  // *.PCD 파일 읽기 
  // https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/sample/region_growing_rgb_passthrough.pcd
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::io::loadPCDFile <pcl::PointXYZRGB> ("region_growing_rgb_passthrough.pcd", *cloud);

  // 알고리즘에서 사용하는 Surface Normal 계산  : Normal 정보 대신 색상 정보를 사용하므로 Normal 계산 작업이 불필요 함 
  
  //오프젝트 생성 
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);         // 입력 
  reg.setSearchMethod (tree);        // 탐색 방법 
  reg.setDistanceThreshold (10);     // 이웃(Neighbor)으로 지정되는 거리 정보 
  reg.setPointColorThreshold (6);    // 동일 Cluter여부를 테스트 하기 위해 사용 (cf. Just as angle threshold is used for testing points normals )
  reg.setRegionColorThreshold (5);   // 동일 Cluter여부를 테스트 하기 위해 사용, 통합(merging)단계에서 사용됨 
  reg.setMinClusterSize (600);       // 최소 포인트수, 지정 값보다 작으면 이웃 포인트와 통합 됨 

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  //파일 저장 
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFile<pcl::PointXYZRGB>("region_growing_rgb_result.pcd", *colored_cloud);

  return (0);
}


```

---

> 원본 코드에 오타가 있습니다. ` pass.setFilterLimits (0.0, 1.0);`  ->  `pass.setFilterLimits (0.0, 5.0);`
