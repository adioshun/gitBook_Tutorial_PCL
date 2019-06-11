# PCL-Cpp 기반 Voxelization

> [Downsampling a PointCloud using a VoxelGrid filter](http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid)



```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());


  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height  //cloud_filtered->points.size()
       << " data points (" << pcl::getFieldsList (*cloud) << ")."<< std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f); //The size of the body is 1 * 1 cm 
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  

  return (0);
}

```


실행 $ 결과 
```
cmake CMakeLists.txt && make && ./Part01-Chapter01 
$ PointCloud before filtering: 460400 data points (x y z intensity distance sid).
$ PointCloud after filtering: 41049 data points (x y z intensity distance sid).

```


시각화 & 결과 

```
$ pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer table_scene_lms400_downsampled.pcd 
```

|![](https://i.imgur.com/yG5GYmm.png)|![](https://i.imgur.com/l8urRKc.png)|
|-|-|
|원본 |원본 확대 |
|![](https://i.imgur.com/OB02KJu.png)|![](https://i.imgur.com/RRGXu4O.png)|
|결과 |결과 확대 |




---
> 다운샘플링은 구조체(PCLPointCloud2)에서만 진행 하여야 하나??

Convert 

```cpp
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>)

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
```


