# PCL-Cpp 기반 노이즈 제거 

## 1. Statistical Outlier Removal

> [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)

> 내부 코드 설명 :[PCL Series 6 - Statistical Filtering (outlier point culling)](https://blog.csdn.net/qq_22170875/article/details/84994029)


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;
  //pcl::io::savePCDFile<pcl::PointXYZ>("passthrough_tabletop.pcd", *cloud_filtered); 

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  //pcl::io::savePCDFile<pcl::PointXYZ>("passthrough_tabletop.pcd", *cloud_filtered); 

  return (0);
}

```

실행 $ 결과
```
$ Loaded 460400
$ Filtered 451410
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

---


## 2. Radius Outlier removal

> [Removing outliers using a Radius Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)

> 내부 코드 : [PCL Series 7 - Radius Filtering (outlier point culling)](https://blog.csdn.net/qq_22170875/article/details/89244371)

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>


int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;


  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);
  // apply filter
  outrem.filter (*cloud_filtered);

  std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  return (0);
}

```


