# PCL-Cpp 기반 ROI Filter

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  
  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);

  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  pcl::io::savePCDFile<pcl::PointXYZ>("passthrough_tabletop.pcd", *cloud_filtered); //Default binary mode save

  return (0);
}


```

---