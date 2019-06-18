# [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)




## 1. using a Matrix4



```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("room_scan1.pcd", *source_cloud);

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix 
  float theta = M_PI/4; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1 (0,3) = 2.5;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);  
  pcl::io::savePCDFile<pcl::PointXYZ>("room_scan1_transformed_cloud.pcd", *transformed_cloud); 

  // Visualization

  return 0;
}

```








---

## 2. Using a Affine3f

- This method is easier and less error prone


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("room_scan1.pcd", *source_cloud);

  float theta = M_PI/4; // The angle of rotation in radians

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);  
  pcl::io::savePCDFile<pcl::PointXYZ>("room_scan1_transformed_cloud.pcd", *transformed_cloud); 

  // Visualization



  return 0;
}

```


---


## Tip. 바로 시각화 

white  = original point cloud
red  = transformed point cloud

```cpp 

  // Visualization

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

```