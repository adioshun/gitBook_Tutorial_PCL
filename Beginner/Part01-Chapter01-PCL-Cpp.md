# [Reading Point Cloud data from PCD files](http://pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)


```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //1. pcl::PointCloud<pcl::PointXYZ> cloud;
  //2. pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud); //내부적으로 reader.read() 호출 
  //1. pcl::io::loadPCDFile<pcl::PointXYZ>("tabletop.pcd", cloud)
  //2. pcl::PCDReader reader;
  //2. reader.read<pcl::PointXYZ>("tabletop.pcd", cloud);

  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl; //cloud_filtered->points.size()

  return (0);
}

```

시각화 확인 :`pcl_viewer tabletop.pcd`

---


# [Writing Point Cloud data to PCD files](http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ> cloud;

// Fill in the cloud data
cloud.width = 5;
cloud.height = 1;
cloud.is_dense = false;
cloud.points.resize (cloud.width * cloud.height);

for (size_t i = 0; i < cloud.points.size (); ++i)
{
cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
}

//내부적으로 writer.write 호출 
pcl::io::savePCDFile<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
//pcl::io::savePCDFileASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
//pcl::io::savePCDFileBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode 

//2. pcl::PCDWriter writer;
//2.1 writer.write<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
//2.2 writer.writeASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
//2.3 writer.writeBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode 
//options. writer.write("test.pcd", *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;


return (0);
}
```