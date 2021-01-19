---
description: '20210119 : 초안 작성'
---

# Chapter02 : Filter

## 2.1. 필터링 모듈을 이용한 불필요한 포인트 클라우드 제거

필터링이란 입력되는 신호나 데이터의 일부 성분을 제거하거나 일부 특성을 변경시키는 작업을 의미 합니다. 2D 이미지 데이터 처리에서는 블러링\(bluring\)/스무딩\(smoothing\)필터를 사용하게되면 영상의 잡음\(noise\)를 제거하고 영상을 부드럽게할 때 사용됩니다. 3D 데이터 처리에서는 불필요한 포인트를 제거 하거나 다운 샘플링을 하여 연산 속도를 높일 때 사용됩니다. 본 문에서는 PCL에서 제공하는 필터링 기능에 대하여 살펴 보겠습니다.

### 2.1.1. 잡음 제거 

모든 데이터 분석의 시작은 이상치\(outlier\) 제거 부터 시작 됩니다. 이상치는 정상치 데이터와 크게 차이가 나서 다른 원인에 의해 생성된 것이 아닌지 의심스러운 데이터를 말합니다. 예를 들어 신용카드 사용내역 데이터에서 평상시 국내 결제가 많은데 갑자기 해외 결제가 발생한다면 이를 이상치로 판단하고 사용자에게 해킹에 위한 사용이 아닌지 알리게 됩니다. 

센서 데이터에서는 잡음\(Noise\)데이터가 이상치에 속할수 있습니다. 잡음 데이터는 관측을 잘못하거나 시스템에서 발생하는 무작위적 오류\(random error\)등에 의해 발생하는 데이터로 대부분의 센서는 특성상 잡음이 포함되어 있으며 제거 하는 작업이 필수 적입니다.

본문에서 사용되는 능동방식의 3D 센서의 경우 2D센서 대비 환경의 영향을 많이 받습니다. 예를 들어 라이다의 경우 먼지는 눈에 잘 보이지는 않지만, 빛에는 탐지되어 반사 될수 있습니다. 

반사 신호들은 실제 물체인지 먼지인지 판단 할수 없기에 모두 점으로 표현 됩니다. 이렇게 생성된 점을 제거 하지 않으면 점군의 특징정보를 왜곡하여 잘못된 탐지 및 인식 결과를 발생 시키므로 사전에 제거 하여야 합니다.

![&#xADF8;&#xB9BC; 2.3 3D &#xB370;&#xC774;&#xD130;&#xC640; &#xB178;&#xC774;&#xC988;](https://user-images.githubusercontent.com/17797922/104986718-05269000-5a57-11eb-925c-04c5c50861ed.png)

\[그림 2.3\]은 책상을 센서로 스캔한것입니다. 노이즈의 예를 확대한 것입니다. 다행히 이러한 노이즈들은 정상적인 point 대비 고유의 특징을 가지고 있어 이를 활용하여 제거 할수 있습니다. PCL에서는 탐지된 점을 주변 점과 비교 하여 실제 물체에서 발생한 점인지 오류로 생성된 점인지 반별 하는 방법을 제공하고 있습니다.

* Statistical based : 통계 정보 기반 점 제거 
* Radius based : 거리 정보 기반 점 제거

#### 통계 기반 잡음 제거

![&#xADF8;&#xB9BC; 2.4 &#xD1B5;&#xACC4;&#xAE30;&#xBC18; &#xC7A1;&#xC74C;&#xC81C;&#xAC70;](https://user-images.githubusercontent.com/17797922/104986822-56cf1a80-5a57-11eb-8aed-cabe2f4cac3c.jpg)

통계 기반 잡음 제거 방법은 각 점과 이웃한 점들간의 통계적 분석을 통해 불규칙성을 판별하는 방법입다. 이 방법은 각점과 이웃한 점의 거리 평균은 가우시안분포를 따른다고 가정하고 특정 임계치값을 넘는 점은 잡음으로 간주 하여 제거 하는 방법입니다. 

동작 과정은 아래와 같습니다.

1. 계산시 참고할 이웃점의 갯수 k를 지정 합니다. 
2. 표준편차 승수\(standard deviation multiplier\) a를 지정 합니다. 
3. 각 점 P\_i의 가장 가까운 이웃점\(P\_TCP\) 찾습니다. 
4. 점 P\_i와 가장 가까운 점 거리 d를 계산 합니다. 
5. P\_i와 이웃한 점\(P\_CNP\)들의 평균 거리u\_d를 계산 합니다. 
6. 거리 d\_i에 대한 표준편차 \(\)를 계산 합니다. 
7. 허용 임계치 T\_g= … 를 계산 합니다. mean + stddev\_mult \* stddev
8. D&gt;T 조건을 만족하는 점을 제거 합니다. 

사용되는 파라미터는 아래와 같습니다.

| 파라미터 | 설명 |
| :--- | :--- |
| setInputCloud | 입력 포인트클라우드 |
| setMean | 분석시 고려할 이웃 점의 수 |
| setStddevMulThresh | Outlier로 처리할 거리 정보 |
| filter | 결과물 포인트클라우 |

setMean는 평균 거리를 계산 할 때 고려할 이웃 점의 갯 수k 입니다. setStddevMulThresh 표준편차에 기반한 임계치 값으로 값이 작을수록 aggressive 하게 적용됩니다. 예를 들어 sK 가 50이고, setStddevMulThresh가 1일 경우 기준점 이웃에 위치한 50개의 점들의 평균 거리의 표준 편차가 1 이상인 거리에 있는 점은 잡음으로 간주 하여 제거 합니다.

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

//Downsampling a PointCloud using a VoxelGrid filter
//http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  // 포인트수 출력
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 오브젝트 생성 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);            //입력 
  sor.setMeanK (50);                    //분석시 고려한 이웃 점 수(50개)
  sor.setStddevMulThresh (1.0);         //Outlier로 처리할 거리 정보 
  sor.filter (*cloud_filtered);         // 필터 적용 
  
  // 생성된 포인트클라우드 수 출력 
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // 생성된 포인트클라우드(inlier) 저장 
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  // 생성된 포인트클라우드(outlier) 저장 
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}

```

출력 결과는 아래와 같습니다. 

```cpp
$ Loaded : 460400 
$ Filtered : 451410
```

입력에 사용된 `table_scene_lms400.pcd` 의 전체 포인트 수는 460400개이며 잡음이 제거 되고 451410만 남게 되었습니다. 

시각화를 통해 결과를 확인해 보기 위해서는 아래 명령어를 입력 하면 됩니다. 

```cpp
pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer StatisticalOutlierRemoval.pcd 
$ pcl_viewer StatisticalOutlierRemoval_Neg.pcd
```



![&#xADF8;&#xB9BC; 2.5 &#xD1B5;&#xACC4;&#xAE30;&#xBC18; &#xC7A1;&#xC74C;&#xC81C;&#xAC70; &#xACB0;&#xACFC;](https://user-images.githubusercontent.com/17797922/104987225-89c5de00-5a58-11eb-92dc-69c6785f58a8.png)

결과 \#1은 원본 이미지에 통계 기반 잡음제거를 적용후 잡음이 아닌 물체로 판단됙 결과\(inlier\)이며, 결과 \#2는 잡음으로 판된되어 제거된 결과\(outlier\)입니다.

#### 반지름 기반 잡음 제거

![&#xADF8;&#xB9BC; 2.6 &#xBC18;&#xC9C0;&#xB984; &#xAE30;&#xBC18; &#xC7A1;&#xC74C;&#xC81C;&#xAC70;](https://user-images.githubusercontent.com/17797922/104987342-d27d9700-5a58-11eb-9a8e-434dbdb9f4ba.png)

반지름 기반 잡음 제거 방법은 가장 간단한 잡음 제거 방법 입니다. 정상적인 점군들은 서로 밀집되어 있고 잡음은 sparse 또는 고립\(isolated\)되어 있다고 가정합니다. 각점의 검색 반경안에 존재 하는 점의 갯수가 특정 임계치를 점지 않을경우 잡음으로 간주 하여 제거 하는 방법이다. 

동작 과정은 아래와 같습니다. 

1. 점의 검색 반경 d을 지정 합니다. 
2. 검색 반경내 최소 포인트 수 Min을 지정합니다. 
3. 각 점 P\_i의 반경 d내 존재 하는 점의 갯수 C를 셉니다. 
4. C &lt; Min 조건을 만족 하는 점을 제거 합니다.

사용되는 파라미터는 아래와 같습니다.

| 파라미터 | 설명 |
| :--- | :--- |
| setInputCloud | 입력 포인트클라우드 |
| setRadiusSearch | 분석시 고려할 반경 범 |
| setMinNeighborsInRadius | Outlier로 처리할 거리 정보 |
| filter | 결과물 포인트클라우 |

setRadiusSearch는 기준점으로부터 탐색할 반경 d 의미 합니다. setMinNeighborsInRadius는 탐색 범위내에 필요한 최소 포인트 수입니다. 예를 들어 d가 0.8이고, setMinNeighborsInRadius가 2이면 0.8m이내에 2개 이상은 잡음으로 간주 됩니다. 그림에서 임계치값이 1일경우 노란점은 잡음으로 간주 되며 2로 설정된다면 노란점과 녹색점은 잡음으로 간주됩니다.

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

// Removing outliers using a Conditional or RadiusOutlier removal
// http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers

int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded : " << cloud->width * cloud->height  << std::endl;

  // 오프젝트 생성 
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);    //입력 
  outrem.setRadiusSearch(0.01);    //탐색 범위 0.01
  outrem.setMinNeighborsInRadius (10); //최소 보유 포인트 수 10개 
  outrem.filter (*cloud_filtered);  // 필터 적용 

  // 포인트수 출력
  std::cout << "Result :  " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  // 생성된 포인트클라우드(inlier) 저장 
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal.pcd", *cloud_filtered, false);

  // 생성된 포인트클라우드(outlier) 저장 
  outrem.setNegative (true);
  outrem.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal_Neg.pcd", *cloud_filtered, false);

  return (0);
}

```



