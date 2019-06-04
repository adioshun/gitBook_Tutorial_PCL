# [Reading Point Cloud data from PCD files](http://pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)


```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  
  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;

  return (0);
}

```

---


# [Writing Point Cloud data to PCD files](http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)