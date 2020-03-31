# Model-Outlier-Removal-PCL-Cpp \(50%\)

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>

// Filtering a PointCloud using ModelOutlierRemoval
// http://pointclouds.org/documentation/tutorials/model_outlier_removal.php#model-outlier-removal

int
main ()
{
  // *.PCD 파일 읽기 
  // https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/sample/sphere_pointcloud_with_noise.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile <pcl::PointXYZ> ("sphere_pointcloud_with_noise.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

  // 2. filter sphere:
  // 2.1 generate model:
  // modelparameter for this sphere:
  // position.x: 0, position.y: 0, position.z:0, radius: 1
  pcl::ModelCoefficients sphere_coeff;
  sphere_coeff.values.resize (4);
  sphere_coeff.values[0] = 0;
  sphere_coeff.values[1] = 0;
  sphere_coeff.values[2] = 0;
  sphere_coeff.values[3] = 1;

  pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
  sphere_filter.setModelCoefficients (sphere_coeff);
  sphere_filter.setThreshold (0.05);
  sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
  sphere_filter.setInputCloud (cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  sphere_filter.filter (*cloud_sphere_filtered);

  std::cerr << "Sphere after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_sphere_filtered->points.size (); ++i)
    std::cout << "    " << cloud_sphere_filtered->points[i].x << " " << cloud_sphere_filtered->points[i].y << " " << cloud_sphere_filtered->points[i].z
        << std::endl;

  return (0);
}
```

