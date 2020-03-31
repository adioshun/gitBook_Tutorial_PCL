# ICP-PCL-Cpp \(70%\)

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *cloud_in);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *cloud_out);


  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}
```

결과

```text
has converged:1 score: 0.131098
   0.998086   0.0580758  -0.0212953  -0.0897905
 -0.0579691    0.998303  0.00559049    0.043843
  0.0215839 -0.00434538    0.999758  -0.0233652
          0           0           0           1
```

