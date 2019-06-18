# PCL-Cpp 기반 업샘플링 

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter02-Upsampling-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[table_scene_lms400_downsampled.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/table_scene_lms400_downsampled.pcd )을 사용하였습니다. 




```cpp

#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

int
main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Read a PCD file from disk.
    pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud);
    std::cout << "Loaded " << cloud->width * cloud->height << std::endl;
    
    // Filtering object.
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Object for searching.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    // Use all neighbors in a radius of 3cm.
    filter.setSearchRadius(0.03);
    // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // Radius around each point, where the local plane will be sampled.
    filter.setUpsamplingRadius(0.03);
    // Sampling step size. Bigger values will yield less (if any) new points.
    filter.setUpsamplingStepSize(0.02);
    
    filter.process(*filteredCloud);
    
    pcl::io::savePCDFile<pcl::PointXYZ>("table_scene_lms400_upsampled.pcd", *filteredCloud);
    std::cout << "Result " << filteredCloud->width * filteredCloud->height << std::endl;
}
```


실행 & 결과 
```
$ Loaded 41049
$ Result 163028
```


시각화 & 결과 

```
$ pcl_viewer table_scene_lms400_downsampled.pcd 
$ pcl_viewer table_scene_lms400_upsampled.pcd
```

|![](https://i.imgur.com/qBIERXw.png)|![](https://i.imgur.com/l5g1BIL.png)|
|-|-|
|원본 |원본 확대 |
|![](https://i.imgur.com/kZpUPrT.png)|![](https://i.imgur.com/THFLb9W.png)|
|결과 |결과 확대 |




