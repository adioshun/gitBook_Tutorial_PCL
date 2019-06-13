

# Neighbors within voxel search

```cpp

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

  // Neighbors within voxel search
  std::vector<int> pointIdxVec;

  if (octree.voxelSearch (searchPoint, pointIdxVec))
  {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x 
     << " " << searchPoint.y 
     << " " << searchPoint.z << ")" 
     << std::endl;
              
    for (size_t i = 0; i < pointIdxVec.size (); ++i)
   std::cout << "    " << cloud->points[pointIdxVec[i]].x 
       << " " << cloud->points[pointIdxVec[i]].y 
       << " " << cloud->points[pointIdxVec[i]].z << std::endl;
  }


}
```

결과 

```
...
-0.00606756 -1.46653 0.797328
-0.00904433 -1.46755 0.796737
-0.0120327 -1.46887 0.795969
...
```

# K nearest neighbor search


```cpp
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

  // K nearest neighbor search

  int K = 10;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }


}

```

결과 

```
K nearest neighbor search at (0.026256 -1.46474 0.929567) with K=10
0.0262559 -1.46474 0.929567 (squared distance: 3.69042e-13)
0.0234182 -1.46435 0.929759 (squared distance: 8.24415e-06)
0.0290953 -1.46517 0.929357 (squared distance: 8.28962e-06)
0.0262519 -1.46476 0.932708 (squared distance: 9.86657e-06)
0.0262599 -1.46472 0.926419 (squared distance: 9.90814e-06)
0.0290885 -1.46518 0.932502 (squared distance: 1.68363e-05)
0.0234196 -1.46433 0.926612 (squared distance: 1.69452e-05)
0.0234169 -1.46437 0.932899 (squared distance: 1.93018e-05)
0.029102 -1.46515 0.926206 (squared distance: 1.95655e-05)
0.0205821 -1.46402 0.929919 (squared distance: 3.28378e-05)
```

## Neighbors within radius search

```cpp
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

```


결과 

```
...
-0.00606756 -1.46653 0.797328
-0.00904433 -1.46755 0.796737
-0.0120327 -1.46887 0.795969
...
```





![](https://i.imgur.com/uFNQCP4.png)