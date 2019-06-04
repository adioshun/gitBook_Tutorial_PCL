# [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)



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






---


# [Removing outliers using a Radius Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)