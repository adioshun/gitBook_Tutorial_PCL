# DetectChanges-PCL-Cpp  \(50%\)

```cpp
#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>

//Spatial change detection on unorganized point cloud data
//http://pointclouds.org/documentation/tutorials/octree_change.php#octree-change-detection
//Commnets : Hunjung, Lim (hunjung.lim@hotmail.com)

int
main (int argc, char** argv)
{

  //복셀 크기 설정(Set octree voxel resolution)
  float resolution = 0.01f; //side length of octree voxels

  // Class 초기화 (Instantiate octree-based point cloud change detection class)
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/RANSAC_plane_false.pcd)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("RANSAC_plane_false.pcd", *cloudA);

  // 포인트 클라우드를 Octree 적용 (Add points from cloudA to octree)
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // 기존 트리 구조는 유지한상태로 Octree 초기화 (Switch octree buffers: This resets octree but keeps previous tree structure in memory.)
  octree.switchBuffers ();

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/tabletop_passthrough.pcd)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("tabletop_passthrough.pcd", *cloudB);


  // 포인트 클라우드를 Octree 적용(Add points from cloudB to octree)
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

  // 이전 포인트클라우드에 존재 하지 않던 새 점군의 Index 저장 (Get vector of point indices from octree voxels which did not exist in previous buffer)
  std::vector<int> newPointIdxVector;
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // 정보 출력 
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
              << cloudB->points[newPointIdxVector[i]].y << " "
              << cloudB->points[newPointIdxVector[i]].z << std::endl;


}
```

