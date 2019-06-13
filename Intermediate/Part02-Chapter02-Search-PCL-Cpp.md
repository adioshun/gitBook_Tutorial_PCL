
## PCL-CPP 기반 Octree 탐색 

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. 



```cpp
#include <pcl/io/pcd_io.h> 
#include <pcl/octree/octree_search.h>  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/point_types.h>  

#include <iostream>
#include <vector>

int main()
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_cluster_0.pcd", *cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}

	//Create an Octree instance object
	float resolution = 0.03f; //Set octree voxel resolution
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); //Create an octree object
	octree.setInputCloud(cloud); 
	octree.addPointsFromInputCloud();  //Build Octree

	//3.1. Voxel Neighbor Search #1
	//pcl::PointXYZRGB searchPoint;
	//searchPoint.x = 0.026256f;
  	//searchPoint.y = -1.464739f;
  	//searchPoint.z = 0.929567f;

	//3.1. Voxel Neighbor Search #2
	pcl::PointXYZRGB searchPoint1 = cloud->points[3000]; //Set the lookup point

	std::vector<int> pointIdxVec;  //Save the result vector of the voxel neighbor search

	if (octree.voxelSearch(searchPoint1, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint1.x
			<< " " << searchPoint1.y
			<< " " << searchPoint1.z << ")"
			<< std::endl;

		//Set the color of the nearest neighbor point
		for (size_t i = 0; i < pointIdxVec.size(); ++i){
			cloud->points[pointIdxVec[i]].r = 255;
			cloud->points[pointIdxVec[i]].g = 0;
			cloud->points[pointIdxVec[i]].b = 0;
		}		
	}

	//3.2.K nearest neighbor search
	pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; 
	int K = 50;
	std::vector<int> pointIdxNKNSearch; //Save the index result of the K nearest neighbor
	std::vector<float> pointNKNSquaredDistance;  //Save the index result of the K nearest neighbor

	std::cout << "K nearest neighbor search at (" << searchPoint2.x
		<< " " << searchPoint2.y
		<< " " << searchPoint2.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{   
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
			cloud->points[pointIdxNKNSearch[i]].r = 0;
			cloud->points[pointIdxNKNSearch[i]].g = 255;
			cloud->points[pointIdxNKNSearch[i]].b = 0;
		}	
	}
	std::cout << "K = 50 nearest neighbors:" << pointIdxNKNSearch.size() << endl;

	//3.3. Neighbor search within radius
	pcl::PointXYZRGB searchPoint3 = cloud->points[1000]; //Set the lookup point
	std::vector<int> pointIdxRadiusSearch;  //Save the index of each neighbor
	std::vector<float> pointRadiusSquaredDistance;  //Save the square of the Euclidean distance between each neighbor and the search point
	float radius = 0.02; //Set the search radius

	std::cout << "Neighbors within radius search at (" << searchPoint3.x
		<< " " << searchPoint3.y
		<< " " << searchPoint3.z
		<< ") with radius=" << radius << std::endl;

	if (octree.radiusSearch(searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{    
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
			cloud->points[pointIdxRadiusSearch[i]].r = 0;
			cloud->points[pointIdxRadiusSearch[i]].g = 0;
			cloud->points[pointIdxRadiusSearch[i]].b = 255;
		}		
	}
	std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << endl;


	pcl::io::savePCDFile<pcl::PointXYZRGB>("Octree_AllinOne.pcd", *cloud);
}



```

결과 

```
Neighbors within voxel search at (0.0346006 -1.46636 0.975463)
K nearest neighbor search at (0.0346006 -1.46636 0.975463) with K=50
K = 50 nearest neighbors:50
Neighbors within radius search at (0.0881522 -1.51977 1.05603) with radius=0.02
Radius 0.02 nearest neighbors: 66
```



|![](https://i.imgur.com/uFNQCP4.png)|![](https://i.imgur.com/OMH7eZs.png)|
|-|-|
|참고위치|결과|




---

# 각 기능별 

## 1. Neighbors within voxel search

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Voxel-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. 

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

## 2. K nearest neighbor search

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Knn-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. 







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

## 3. Neighbors within radius search

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Radius-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. 


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





