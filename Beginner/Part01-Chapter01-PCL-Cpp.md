# PCL-Cpp 기반 I/O


PCL에서는 점군 입출력을 위해서 1개의 템플릿 클래스와 4개의 구조체를 지원 하고 있습니다. 자세한 내용은 [[이곳]](https://www.twblogs.net/a/5c27931ebd9eee16b3dbc3eb)을 참고 하시면 됩니다. 

```cpp
// template
pcl::PointCloud<pcl::PointXYZ>

// struct 
Pcl::PointXYZ
Pcl::PointXYZI
Pcl::PointXYZRGB
Pcl::PCLPointCloud2
```

## 1. 읽기 

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  
  //READ #1 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud); //내부적으로 reader.read() 호출 

  //READ #2
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::io::loadPCDFile<pcl::PointXYZ>("tabletop.pcd", cloud) //내부적으로 reader.read() 호출 

  //READ #3
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCDReader reader;
  //reader.read<pcl::PointXYZ>("tabletop.pcd", cloud);


  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl; //cloud_filtered->points.size()

  return (0);
}

```

시각화 확인 
```
$ pcl_viewer tabletop.pcd
```

---


## 2. 생성 & 쓰기 


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //SAVE #1
  #pcl::PointCloud<pcl::PointXYZ> cloud; //SAVE #2

  
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
  
  
  //SAVE #1
  pcl::io::savePCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud); //Default binary mode save


  //SAVE #2
  //내부적으로 writer.write 호출
  //pcl::io::savePCDFile<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
  //pcl::io::savePCDFileASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
  //pcl::io::savePCDFileBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode 
  


    
  //SAVE #3
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
  //writer.writeASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
  //writer.writeBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode
  //options. writer.write("test.pcd", *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
  
  
  return (0);
}
```



---

## [Replace each other ](https://blog.csdn.net/qq_16481211/article/details/85332763#pclPCLPointCloud2_80)

```cpp
// Converting pcl::PCLPointCloud2 to pcl::PointCloud and reverse
#include <pcl/conversions.h>
pcl::PCLPointCloud2 point_cloud2;
pcl::PointCloud<pcl::PointXYZ> point_cloud;
pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
pcl::toPCLPointCloud2(point_cloud, point_cloud2);
```
---
- [Reading Point Cloud data from PCD files](http://www.pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)

- [Writing Point Cloud data to PCD files](http://www.pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)












