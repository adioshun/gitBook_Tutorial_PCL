# PCL-Cpp 기반 바닥제거(RANSAC)

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[tabletop_passthrough.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)을 사용하였습니다. 원본 코드는 [[이곳]](http://pointclouds.org/documentation/tutorials/extract_indices.php)을 참고 하였습니다.


```cpp
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

//Plane model segmentation
//http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


  // 오프젝트 생성 
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);       //(옵션)
  seg.setInputCloud (cloud);                 //입력 
  seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델 
  seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법 
  seg.setMaxIterations (1000);               //최대 실행 수
  seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보 
  seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 
 

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  //추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  
  num_loop = 
  for (std::size_t i = 0; i < inliers->indices.size (); ++i)
  std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                             << cloud->points[inliers->indices[i]].y << " "
                                             << cloud->points[inliers->indices[i]].z << std::endl;


  // Create the filtering object
  //Extracting indices from a PointCloud
//http://pointclouds.org/documentation/tutorials/extract_indices.php
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
 extract.setInputCloud (cloud);
 extract.setIndices (inliers);
 extract.setNegative (true);//false
 extract.filter (*cloud_p);
 std::cerr << "Filtered : " << cloud_p->width * cloud_p->height << " data points." << std::endl;

 std::stringstream ss;
 ss << "RANSAC_plane_true.pcd"; //RANSAC_plane_false.pcd
 pcl::PCDWriter writer2;
 writer2.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);


  return (0);

 
}

```

실행 결과 
```
Point cloud data: 15 points
    0.352222 -0.151883 2
    -0.106395 -0.397406 1
    -0.473106 0.292602 1
    -0.731898 0.667105 -2
[pcl::SACSegmentation::initSAC] Setting the maximum number of iterations to 50
Model coefficients: 0 0 1 -1
Model inliers: 12
1    -0.106395 -0.397406 1
2    -0.473106 0.292602 1
4    0.441304 -0.734766 1
```

시각화 & 결과

```
$ pcl_viewer tabletop_passthrough.pcd 
$ pcl_viewer RANSAC_plane.pcd  
$ pcl_viewer RANSAC_plane_true.pcd
```









|![](https://i.imgur.com/qhczRfW.png)|![](https://i.imgur.com/Upo7BZK.png)|![](https://i.imgur.com/j6HoJBy.png)|
|-|-|-|
|원본`tabletop_passthrough.pcd` | 결과`setNegative (false)` |결과 `setNegative (true)`|
