# Normal-PCL-Cpp \(70%\)

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Read a PCD file from disk.
    pcl::io::loadPCDFile<pcl::PointXYZ>("lobby.pcd", *cloud);

    // Object for normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // For every point, use all neighbors in a radius of 3cm.
    normalEstimation.setRadiusSearch(0.03);
    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);

    // Calculate the normals.
    normalEstimation.compute(*normals);

    // Visualize them.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // Display one normal out of 20, as a line of length 3cm.
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 20, 0.03, "normals");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
```

normals are stored in "PointCloud" objects

`setRadiusSearch()`: `setKSearch(int K)`: point's K nearest neighbors to compute the normal

> 추후 Normal 시작화 방법 추가

#### Normal 생성 + 병합 + 저장 

```cpp
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

// How 3D Features work in PCL
// http://pointclouds.org/documentation/tutorials/how_features_work.php

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // 입력 포인트 클라우드 저장할 오브젝트 
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); // 계산된 Normal을 저장할 오브젝트 
  pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  std::cout << "INPUT " << cloud->points.size ()  << std::endl;
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);  
  ne.setRadiusSearch (0.03); // Use all neighbors in a sphere of radius 3cm  
                             // setKSearch() 변경 가능 
  ne.compute (*cloud_normals); // Compute the features

  // 포인트수 출력  
  std::cout << "NORMAL " << cloud_normals->points.size ()  << std::endl;

  // Copy the point cloud data
  pcl::concatenateFields (*cloud, *cloud_normals, p_n_cloud_c);
  pcl::io::savePCDFile<pcl::PointNormal>("p_n_cloud_c.pcd", p_n_cloud_c);

  std::cerr << "Cloud C: " << std::endl;
  for (std::size_t i = 0; i < 5; ++i)
    std::cerr << "    " <<
      p_n_cloud_c[i].x << " " << p_n_cloud_c[i].y << " " << p_n_cloud_c[i].z << " " <<
      p_n_cloud_c[i].normal[0] << " " << p_n_cloud_c[i].normal[1] << " " << p_n_cloud_c[i].normal[2] << std::endl;



  // Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 20, 0.03, "normals");
	while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
```

