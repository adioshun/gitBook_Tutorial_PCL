# PCL-Cpp를 이용하 시각화 

두가지 방법 제공 
- Cloud Viewer : 간단한 기능만을 가진 뷰어 [[참고]](https://adioshun.gitbooks.io/pcl/content/visualization/visualizing-point-clouds.html)
- Pcl Viewer : playing normals, drawing shapes, multiple viewport의 기능 가짐 [[참고]](https://adioshun.gitbooks.io/pcl/content/visualization/pclvisualizer.html)


## Cloud Viewer를 활용한 코드내 시각화 기능 추가 

```cpp 
#include <pcl/visualization/cloud_viewer.h> 

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

//...
pcl::visualization::CloudViewer viewer("cloud viewer");
viewer.showCloud(cloud1);

system("pause");

```

## Pcl Viewer를 이용한 PCD파일 시각화

```
$ sudo apt install pcl-tools 
$ pcl_viewer [파일명.pcd]
```
