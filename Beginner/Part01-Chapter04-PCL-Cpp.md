# [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)

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

---

# [Removing outliers using a Conditional Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)


```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>


int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;


  // build the condition
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
    pcl::ConditionAnd<pcl::PointXYZ> ());

  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));

  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter (*cloud_filtered);
  
 
  std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  return (0);
}
```



---


# [Removing outliers using a Radius Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)

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


