# PCL-Cpp 기반 Voxelization

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter02-Downsampling-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[table_scene_lms400.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/table_scene_lms400.pcd )을 사용하였습니다. 


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Downsampling a PointCloud using a VoxelGrid filter
//http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
    pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud); 


  // 읽은 table_scene_lms400.pcd 파일의 포인트수 출력
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // 오브젝트 생성 
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);              //입력
  sor.setLeafSize (0.01f, 0.01f, 0.01f); //leaf size  1cm 
  sor.filter (*cloud_filtered);          //출력 

  // 생성된 포인트클라우드 수 출력 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  // 생성된 포인트클라우드 저장 
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
```


실행 & 결과 
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

- [Downsampling a PointCloud using a VoxelGrid filter](http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid)






> 다운샘플링은 구조체(PCLPointCloud2)에서만 진행 하여야 하나??

Convert 

```cpp
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>)

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
```


