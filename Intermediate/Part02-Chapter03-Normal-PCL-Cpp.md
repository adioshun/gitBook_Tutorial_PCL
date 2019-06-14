# PCL-Cpp 기반 Normal 계산 


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

`setRadiusSearch()`: 
`setKSearch(int K)`: point's K nearest neighbors to compute the normal


> 추후 Normal 시작화 방법 추가 