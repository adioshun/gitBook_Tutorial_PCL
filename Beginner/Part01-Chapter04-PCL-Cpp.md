# PCL-Cpp 기반 노이즈 제거 

## 1. Statistical Outlier Removal

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter04-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[table_scene_lms400.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/table_scene_lms400.pcd )을 사용하였습니다. 


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Removing outliers using a StatisticalOutlierRemoval filter
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  // 포인트수 출력
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 오브젝트 생성 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);            //입력 
  sor.setMeanK (50);                    //분석시 고려한 이웃 점 수(50개)
  sor.setStddevMulThresh (1.0);         //Outlier로 처리할 거리 정보 
  sor.filter (*cloud_filtered);         // 필터 적용 
  
  // 생성된 포인트클라우드 수 출력 
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // 생성된 포인트클라우드(inlier) 저장 
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  // 생성된 포인트클라우드(outlier) 저장 
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}


```

실행 & 결과
```
$ Loaded : 460400
$ Filtered : 451410
```


시각화 & 결과

```
$ pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer StatisticalOutlierRemoval.pcd 
$ pcl_viewer StatisticalOutlierRemoval_Neg.pcd 
```



|![](https://i.imgur.com/yn4JEuH.png)|![](https://i.imgur.com/eSJIQlT.png)|![](https://i.imgur.com/92kPpnC.png)|
|-|-|-|
|원본|결과|결과(SetNegarive)|

|![](https://i.imgur.com/R4GPZI7.png)|![](https://i.imgur.com/IM34sRA.png)|
|-|-|
|원본(확대)|결과(확대)|

---


## 2. Radius Outlier removal



```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

// Removing outliers using a Conditional or RadiusOutlier removal
// http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers

int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded : " << cloud->width * cloud->height  << std::endl;

  // 오프젝트 생성 
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);    //입력 
  outrem.setRadiusSearch(0.01);    //탐색 범위 0.01
  outrem.setMinNeighborsInRadius (10); //최소 보유 포인트 수 10개 
  outrem.filter (*cloud_filtered);  // 필터 적용 

  // 포인트수 출력
  std::cout << "Result :  " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  // 생성된 포인트클라우드(inlier) 저장 
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal.pcd", *cloud_filtered, false);

  // 생성된 포인트클라우드(outlier) 저장 
  outrem.setNegative (true);
  outrem.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal_Neg.pcd", *cloud_filtered, false);

  return (0);
}

```

실행 & 결과
```
$ Loaded : 460400
$ Result : 455495
```


시각화 & 결과

```
$ pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer Radius_Outlier_Removal.pcd 
$ pcl_viewer Radius_Outlier_Removal_Neg.pcd 
```



|![](https://i.imgur.com/yn4JEuH.png)|![](https://i.imgur.com/eSJIQlT.png)|![](https://i.imgur.com/ZFwFOHh.png)|
|-|-|-|
|원본|결과|결과(SetNegarive)|


---

> [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)

> 내부 코드 설명 :[PCL Series 6 - Statistical Filtering (outlier point culling)](https://blog.csdn.net/qq_22170875/article/details/84994029)

> [Removing outliers using a Radius Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)

> 내부 코드 : [PCL Series 7 - Radius Filtering (outlier point culling)](https://blog.csdn.net/qq_22170875/article/details/89244371)



