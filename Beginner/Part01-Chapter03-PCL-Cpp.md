# PCL-Cpp 기반 ROI 추출 

## 1. PassThrough 

> [Filtering a PointCloud using a PassThrough filter](http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough)

```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);

  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.70, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;  

  pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_passthrough.pcd", *cloud_filtered); //Default binary mode save

  return (0);
}

```


실행 $ 결과
```
$ Loaded :202627
$ Filtered :72823
```

시각화 & 결과

```
$ pcl_viewer tabletop.pcd 
$ pcl_viewer tabletop_passthrough.pcd 
```

|![](https://i.imgur.com/tZzHIRS.png)|![](https://i.imgur.com/hpfXFql.png)|
|-|-|
|원본|결과|


---


## 2. Conditional Outlier removal

>  [Removing outliers using a Conditional Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)


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



