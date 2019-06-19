# PCL-Cpp 기반 Smoothing


```cpp

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile ("bun0.pcd", *cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls; 
  mls.setComputeNormals (true); //optional
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03); // Use all neighbors in a radius of 3cm.
	//filter.setPolynomialFit(true); //If true, the surface and normal are approximated using a polynomial estimation
                                   // (if false, only a tangent one).
  // Reconstruct
  mls.process (*smoothedCloud);

  // Save output
  pcl::io::savePCDFile ("bun0-mls.pcd",*smoothedCloud);
}


```