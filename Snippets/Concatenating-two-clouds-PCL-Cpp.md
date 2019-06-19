# Concatenating two clouds

> [Concatenate the points of two Point Clouds](http://pointclouds.org/documentation/tutorials/concatenate_clouds.php)

점군 Concatenating에는 두가지 종류가 있습니다. 
- Point : 두개의 점군(`cloud_A.pcd`, `cloud_B.pcd`)을 합쳐서 포인트의 수를 늘리는 것입니다. 합치기 위해서 두 점군은 동일하여야 합니다. XYZ와 XYZRGB는 합칠수 없습니다. 
- Fields : 서로 다른 종류의 점군을 합쳐서 정보를 늘리는 것입니다. XYZ와 Normal을 합쳐서 XYZNormal을 만들수 있습니다. 합칠때 두 점군의 수는 동일해야 합니다. XYZ가 10개의 포인트를 가지고 있다면, Normal도 10개의 정보를 가지고있어야 합니다. 


---

## 1. Point Concatenating

덧셈(`+`) 연산으로 쉽게 수행 가능 합니다. 

```cpp
cloud_c = cloud_a;
cloud_c += cloud_b;
```

```cpp 
#include <pcl/io/pcd_io.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZ>);

	// Read two PCD files from disk.
	pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_A.pcd", *cloudA);
	pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_B.pcd", *cloudB);
	
	// Create cloud "C", with the points of both "A" and "B".
	*cloudC = (*cloudA) + (*cloudB);
	pcl::io::savePCDFile<pcl::PointXYZ>("cloud_C.pcd", *cloudC); 
}
```

---

## 2. Fileds Concatenating

`pcl::concatenateFields()`로 수행 가능 합니다. 

```cpp
concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c)
```

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointNormal>);

	// Read a PCD file from disk.
	pcl::io::loadPCDFile<pcl::PointXYZ>("tabletop.pcd", *cloudPoints);
	
	// Compute the normals of the cloud 
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloudPoints);
	normalEstimation.setRadiusSearch(0.05);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*cloudNormals);

	// Concatenate the fields (PointXYZ + Normal = PointNormal).
	pcl::concatenateFields(*cloudPoints, *cloudNormals, *cloudAll);


	std::cout << "Point:" << std::endl;
	std::cout << "\tXYZ:" << cloudAll->points[0].x << " "
				<< cloudAll->points[0].y << " "
				<< cloudAll->points[0].z << std::endl;
	std::cout << "\tNormal:" << cloudAll->points[0].normal[0] << " "
				<< cloudAll->points[0].normal[1] << " "
				<< cloudAll->points[0].normal[2] << std::endl;

}
```


실행 결과 
```
Point:
	XYZ:0.0381132 -1.60915 1.08705
	Normal:0.00102124 0.0252977 -0.999679
```

