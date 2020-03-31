# PCL-Cpp  \(70%\)

> 코드는 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [\[tabletop\_passthrough.pcd\]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)을 사용하였습니다. 원본 코드는 [\[이곳\]](http://pointclouds.org/documentation/tutorials/extract_indices.php)을 참고 하였습니다.

```cpp
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Plane model segmentation
//http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        inlierPoints (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZRGB>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // Object for storing the plane model coefficients.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


  // 오프젝트 생성 Create the segmentation object.
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud);                 //입력 
  seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
  seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
  seg.setMaxIterations (1000);               //최대 실행 수
  seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  //seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
  seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 


  //추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *inlierPoints);
  pcl::io::savePCDFile<pcl::PointXYZRGB>("SACSegmentation_result.pcd", *inlierPoints);


 //[옵션]] 바닥 제거 결과 얻기 
 //Extracting indices from a PointCloud
 //http://pointclouds.org/documentation/tutorials/extract_indices.php
 pcl::ExtractIndices<pcl::PointXYZRGB> extract;
 extract.setInputCloud (cloud);
 extract.setIndices (inliers);
 extract.setNegative (true);//false
 extract.filter (*inlierPoints_neg);
 pcl::io::savePCDFile<pcl::PointXYZRGB>("SACSegmentation_result_neg.pcd", *inlierPoints_neg);

  return (0);


}
```

실행 결과

```text
Loaded :72823
Model coefficients: 3.88368e-05 0.000606678 1 -0.773654
```

시각화 & 결과

```text
$ pcl_viewer tabletop_passthrough.pcd 
$ pcl_viewer SACSegmentation_result.pcd  
$ pcl_viewer SACSegmentation_result_neg.pcd
```

| ![](https://i.imgur.com/qhczRfW.png) | ![](https://i.imgur.com/Upo7BZK.png) | ![](https://i.imgur.com/j6HoJBy.png) |
| :--- | :--- | :--- |
| 원본`tabletop_passthrough.pcd` | 결과`setNegative (false)` | 결과 `setNegative (true)` |

