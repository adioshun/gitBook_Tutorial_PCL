# PCL-Cpp 기반 Smoothing


>  코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter06-Smoothig-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[bunny.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/bunny.pcd)을 사용하였습니다.  


```cpp

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile ("bunny.pcd", *cloud);

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
  pcl::io::savePCDFile ("bunny-mls.pcd",*smoothedCloud);
}


```