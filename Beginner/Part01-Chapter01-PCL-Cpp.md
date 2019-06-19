# PCL-Cpp 기반 I/O

## 1. 읽기 

> [Reading Point Cloud data from PCD files](http://pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)


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

> [Writing Point Cloud data to PCD files](http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)


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

# structures

PCL에서는 점군 입출력을 위해서 1개의 템플릿 클래스와 4개의 구조체를 지원 하고 있습니다. 

```
Pcl::PointCloud

Pcl::PointXYZ
Pcl::PointXYZI
Pcl::PointXYZRGB
Pcl::PCLPointCloud2
```

##### 1. Template class : `Pcl::PointCloud`

member variables
- width 
    - unstructured : number of points in the cloud 
    - structured : number of points on **a line** of the point cloud data set, RGB-D등 이미지 camera 
- height 
    - unstructured : 항상 `1`, 따라서 1이 아니면 structured 
    - structured : the total number of rows of the point cloud
    
템플릿 
- `pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);`
- Read RGB: `pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);`
- Read pure XYZ: `pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);`

##### 2. Structure : `PointXYZ`, `PointXYZI`, `PointXYZRGB`, `PCLPointCloud2`


pcl::PointXYZ::PointXYZ 

```cpp
  pcl::PointXYZ::PointXYZ ( float_x,
                      float_y,
                      float_z
                        ) 
```

Pcl::PointXYZI structure

```cpp

union {
   struct {
      float   intensity
   } 	
   float   data_c [4]
}; 

```
Pcl::PointXYZRGB structure

```cpp

Eigen::Vector3i 	getRGBVector3i ()
const Eigen::Vector3i 	getRGBVector3i () const
Eigen::Vector4i 	getRGBVector4i ()
const Eigen::Vector4i 	getRGBVector4i () const
 	PointXYZRGB (const _PointXYZRGB &p)
 	PointXYZRGB ()
 	PointXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
//其中PointXYZRGB	为
pcl::PointXYZRGB::PointXYZRGB	(	uint8_t 	_r,
uint8_t 	_g,
uint8_t 	_b 
)	
	
```

Pcl::PCLPointCloud2 structure

```cpp
struct PCLPointCloud2
         {
          PCLPointCloud2 () : header (), height (0), width (0), fields (),
         is_bigendian (false), point_step (0), row_step (0),
          data (), is_dense (false)
          {
         #if defined(BOOST_BIG_ENDIAN)
          is_bigendian = true;
         #elif defined(BOOST_LITTLE_ENDIAN)
          is_bigendian = false;
         #else
         #error "unable to determine system endianness"
         #endif
          } 
      ::pcl::PCLHeader header;
     
      pcl::uint32_t height;
      pcl::uint32_t width;
     
      std::vector< ::pcl::PCLPointField> fields;
     
      pcl::uint8_t is_bigendian;
      pcl::uint32_t point_step;
      pcl::uint32_t row_step; 
      std::vector<pcl::uint8_t> data; 
      pcl::uint8_t is_dense; 
      public:
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2> Ptr;
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2 const> ConstPtr;
      }; // struct PCLPointCloud2 

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

- [Common data structures in PCL](https://www.twblogs.net/a/5c27931ebd9eee16b3dbc3eb) 










