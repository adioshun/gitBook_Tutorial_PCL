# Euclidean-PCL-Cpp  \(70%\)

> 코드는 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter01-Euclidean-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [\[RANSAC\_plane\_true.pcd\]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/RANSAC_plane_true.pcd)을 사용하였습니다.

```cpp
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Euclidean Cluster Extraction
// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

int 
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/RANSAC_plane_true.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("RANSAC_plane_true.pcd", *cloud);

  // 포인트수 출력
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);  //KdTree 생성 


  std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
  // 군집화 오브젝트 생성  
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setInputCloud (cloud);       // 입력   
  ec.setClusterTolerance (0.02);  // 2cm  
  ec.setMinClusterSize (100);     // 최소 포인트 수 
  ec.setMaxClusterSize (25000);   // 최대 포인트 수
  ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
  ec.extract (cluster_indices);   // 군집화 적용 

  // 클러스터별 정보 수집, 출력, 저장 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     cloud_cluster->points.push_back (cloud->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // 포인트수 출력
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    // 클러스터별 이름 생성 및 저장 
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
```

실행 & 결과

```text
$ PointCloud before filtering has: 23330 data points.
$ PointCloud representing the Cluster: 5981 data points.
$ PointCloud representing the Cluster: 5111 data points.
$ PointCloud representing the Cluster: 4431 data points.
$ PointCloud representing the Cluster: 2768 data points.
$ PointCloud representing the Cluster: 2513 data points.
$ PointCloud representing the Cluster: 1552 data points.
$ PointCloud representing the Cluster: 934 data points.
```

시각화 & 결과

```text
$ pcl_viewer cloud_cluster_0.pcd 
$ pcl_viewer cloud_cluster_6.pcd
```

| ![](https://i.imgur.com/j6HoJBy.png) | ![](https://i.imgur.com/4ZXd2eu.png) | ![](https://i.imgur.com/6WVsQwo.png) |
| :--- | :--- | :--- |
| 원본\(`RANSAC_plane_true.pcd`\) | 결과\(`cloud_cluster_0.pcd`\) | 결과\(`cloud_cluster_6.pcd`\) |

* [Adaptive clustering](https://github.com/yzrobot/adaptive_clustering): `Online learning for human classification in 3D LiDAR-based tracking(2017)` 에서 활용한 Euclidean clustering 기법 

