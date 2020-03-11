# PCL-Cpp 기반 관심영역 설정 

## 1. PassThrough 

> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter03-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[tabletop.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)을 사용하였습니다. 

```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//Filtering a PointCloud using a PassThrough filter
//http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // 오브젝트 생성 
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);                //입력 
  pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
  pass.setFilterLimits (0.70, 1.5);          //적용할 값 (최소, 최대 값)
  //pass.setFilterLimitsNegative (true);     //적용할 값 외 
  pass.filter (*cloud_filtered);             //필터 적용 

  // 포인트수 출력
  std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;  

  // 저장 
  pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_passthrough.pcd", *cloud_filtered); //Default binary mode save

  return (0);
}
```


실행 & 결과
```
$ Loaded :202627
$ Filtered :72823
```

시각화 & 결과

```
$ pcl_viewer tabletop.pcd 
$ pcl_viewer tabletop_passthrough.pcd 
```

|![](https://i.imgur.com/tZzHIRS.png)|![](https://i.imgur.com/hpfXFql.png)|![](https://i.imgur.com/aPux8Az.png)|
|-|-|-|
|원본|결과|결과(Negative)|


---


## 2. Conditional Outlier removal




```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

//Removing outliers using a Conditional or RadiusOutlier removal
//http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers

int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl;

  // 조건 정의 
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new //조건 1 
     pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));  //eg. z축으로 0.00보다 큰값(GT:Greater Than)
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new //조건 2 
     pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.8)));  //eg. z축으로 0.08보다 작은값(LT:Less Than)

  //오프젝트 생성 
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setInputCloud (cloud);        //입력 
  condrem.setCondition (range_cond);    //조건 설정  
  condrem.setKeepOrganized(true);       //
  condrem.filter (*cloud_filtered);     //필터 적용 

  // 포인트수 출력  
  std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  // 저장 
  pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_conditional.pcd", *cloud_filtered); //Default binary mode save
  return (0);
}
```
|![](https://i.imgur.com/tZzHIRS.png)|![](https://i.imgur.com/laPyTLC.png)|
|-|-|
|원본|결과|




