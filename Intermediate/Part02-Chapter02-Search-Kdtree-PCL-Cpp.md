# PCL-Cpp 기반 KdTree 탐색 


> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Kdtree-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. 





```cpp

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_cluster_0.pcd", *cloud);

  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 255;
  cloud->points[i].g = 255;
  cloud->points[i].b = 255;
  }


  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);  


  //K nearest neighbor search
  pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; //Set the lookup point
  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint2.x 
            << " " << searchPoint2.y 
            << " " << searchPoint2.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 255;
      cloud->points[pointIdxNKNSearch[i]].b = 0;
    }
  }
  std::cout << "K = 10 ：" << pointIdxNKNSearch.size() << std::endl;


  // Neighbors within radius search
  pcl::PointXYZRGB searchPoint3 = cloud->points[1000]; //Set the lookup point
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = 0.02; //Set the search radius 

  std::cout << "Neighbors within radius search at (" << searchPoint3.x 
            << " " << searchPoint3.y 
            << " " << searchPoint3.z
            << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
        cloud->points[pointIdxRadiusSearch[i]].r = 0;
        cloud->points[pointIdxRadiusSearch[i]].g = 0;
        cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }
  }

  std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << std::endl;


  pcl::io::savePCDFile<pcl::PointXYZRGB>("Kdtree_AllinOne.pcd", *cloud);

  return 0;
}
```






결과 

```
K nearest neighbor search at (0.0346006 -1.46636 0.975463) with K=10
K = 10 ：10
Neighbors within radius search at (0.0881522 -1.51977 1.05603) with radius=0.02
Radius 0.02 nearest neighbors: 66

```



|![](https://i.imgur.com/uFNQCP4.png)|![](https://i.imgur.com/948C7HR.png)|
|-|-|
|참고위치|결과|




