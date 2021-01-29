---
description: '20210125 : 초안 작성 中 (미완료)'
---

# Chapter03 : Sample Consensus

## 3.1. 모델 매칭을 점군내 모형 탐지 : SAC 모듈

공간에 존재 하는 물체들은 특정한 형태를 가지고 있습니다. 사람과 같이 복잡한 형태도 있지만 바닥\(평면\), 전봇대\(원통\), 간판\(사각형\), 전선\(선\) 등 간단한 형태도 있습니다. 본문에서는 점군속에 포함된 포형을 탐지 하는 법을 살펴 보겠습니다. 탐지 결과는 후에 배우는 세그멘테이션과 연계 하여 불필요한 부분을 제거하거나, 필요한 부분만 추출 할 때 사용 됩니다.

![&#xADF8;&#xB9BC; 3.1 &#xACF5;&#xAC04;&#xC0C1; &#xC874;&#xC7AC; &#xD558;&#xB294; &#xB2E4;&#xC591;&#xD55C; &#xBAA8;&#xB378;\(&#xC120;, &#xC6D0;&#xD1B5;\)](https://user-images.githubusercontent.com/17797922/105662170-685e6980-5f12-11eb-8983-e2a854ce2ac0.png)

### 3.1.1. Sample Consensus 

Sample Consensus는 포인트 클라우드 샘플\(Sample\)이 사전에 정의된 모형\(=모델\)과 일치\(Consensus\)하는지를 판악하여 모델의 파라미터를 추정하는 알고리즘 입니다. 간단한 형태의 도형들은 수학적으로 모델링이 가능합니다. 예를 들어 선의 경우 y=ax+b로 표현 가능합니다. 원의 경우 xxxx로 표현 가능합니다. 수학적 모델을 통해서 점군상에 존재 하는 모형을 찾는데는 허프만 변환과 sample\_consensus 방법이 적용 가능합니다. 허프만 변환은 알고리즘이 간단하여 속도가 빠른 장점이 있지만, 잡음이 섞여 있는 데이터에는 적합 하지 않아 sample\_consensus 방법을 많이 사용합니다. 

PCL에서 구현되어 있는 sample consensus estimators 은 아래와 같습니다. 

* SAC\_RANSAC - RANdom SAmple Consensus 
* SAC\_LMEDS - Least Median of Squares 
* SAC\_MSAC - M-Estimator SAmple Consensus 
* SAC\_RRANSAC - Randomized RANSAC 
* SAC\_RMSAC - Randomized MSAC 
* SAC\_MLESAC - Maximum LikeLihood Estimation SAmple Consensus 
* SAC\_PROSAC - PROgressive SAmple Consensus

PCL에서 제공하는 모형을 표로 정리 하면 아래와 같습니다.

![PCL&#xC81C;&#xACF5; &#xBAA8;&#xB378;](https://user-images.githubusercontent.com/17797922/105662473-23870280-5f13-11eb-9dd3-68a7c4e0b86e.png)

모델 중에서 면을 이용하여 바닥과 벽을 탐지 할 수 있으며, 원통을 이용하여 컵, 기둥, 배관 등을 탐지 할 수 있습니다.

![&#xADF8;&#xB9BC; 3-2 &#xC810;&#xAD70;&#xC73C;&#xB85C; &#xD45C;&#xD604;&#xB41C; &#xB2E4;&#xC591;&#xD55C; &#xBAA8;&#xD615;](https://user-images.githubusercontent.com/17797922/105662616-7e205e80-5f13-11eb-9a44-6adec11a7fbb.png)



### 3.1.2 RANSAC\(RANdom SAmple Consensus\)

RANSAC은 속도는 느리지만 잡음에 강건한 성질을 가지고 있습니다.

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>

// Filtering a PointCloud using ModelOutlierRemoval
// http://pointclouds.org/documentation/tutorials/model_outlier_removal.php#model-outlier-removal


int
main ()
{
  // *.PCD 파일 읽기 
  // https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/sample/sphere_pointcloud_with_noise.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile <pcl::PointXYZ> ("sphere_pointcloud_with_noise.pcd", *cloud);


  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;


  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;


  // 2. filter sphere:
  // 2.1 generate model:
  // modelparameter for this sphere:
  // position.x: 0, position.y: 0, position.z:0, radius: 1
  pcl::ModelCoefficients sphere_coeff;
  sphere_coeff.values.resize (4);
  sphere_coeff.values[0] = 0;
  sphere_coeff.values[1] = 0;
  sphere_coeff.values[2] = 0;
  sphere_coeff.values[3] = 1;


  pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
  sphere_filter.setModelCoefficients (sphere_coeff);
  sphere_filter.setThreshold (0.05);
  sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
  sphere_filter.setInputCloud (cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  sphere_filter.filter (*cloud_sphere_filtered);


  std::cerr << "Sphere after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_sphere_filtered->points.size (); ++i)
    std::cout << "    " << cloud_sphere_filtered->points[i].x << " " << cloud_sphere_filtered->points[i].y << " " << cloud_sphere_filtered->points[i].z
        << std::endl;


  return (0);
}

```

