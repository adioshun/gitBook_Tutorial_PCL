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
Loaded : 460400rFiltered : 451410
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

출력 결과는 아래와 같습니다. 

```cpp
Loaded : 460400 
Result : 455495
```

입력에 사용된 `table_scene_lms400.pcd` 의 전체 포인트 수는 460400개이며 잡음이 제거 되고 455495만 남게 되었습니다. 

시각화를 통해 결과를 확인해 보기 위해서는 아래 명령어를 입력 하면 됩니다. 

```cpp
$ pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer StatisticalOutlierRemoval.pcd 
$ pcl_viewer StatisticalOutlierRemoval_Neg.pcd
```

![&#xADF8;&#xB9BC; 2.7 &#xBC18;&#xC9C0;&#xB984; &#xAE30;&#xBC18; &#xC7A1;&#xC74C;&#xC81C;&#xAC70; &#xACB0;&#xACFC;](https://user-images.githubusercontent.com/17797922/104987888-0d33ff00-5a5a-11eb-8236-fb74f39d6946.png)

결과 \#1은 원본 이미지에 통계 기반 잡음제거를 적용 후 잡음이 아닌 물체로 판단된 결과\(inlier\)이며, 결과 \#2는 잡음으로 판단되어 제거된 결과\(outlier\)입니다.

### 2.1.2. 다운 샘플링

3D 센서를 통해 수집되는 포인트 클라우드 데이터는 센서의 성능, 탐지 물체의 크기와 센서와의 거리, 수집되는 공간의 환경에 따라서 수바이트\(Byte\)에서 수기가바이트\(GByte\)에 이룰 수 있습니다. 따라서 목적에 따라 포인트 클라우들의 수를 줄이거나 늘리는 작업이 필요 합니다. 

다운샘플링이란 센서에서 수집된 포인트 클라우드의 수를 줄이는 것을 의미하며 일반적으로 연산 부하 감소를 목적으로 수행합니다. 예를 들어 Kinect센서의 경우 프레임당 307,000 점군을 생성하고 매초 45MByte처리 해야 한다\(Moreno & Li, 2016\). 자율 주행차에서 사용되는 라이다의 경우 매 0.1초마다 데이터를 수집되며 약 수십만개의 점군으로 구성되어 있어 분석을 위해서는 큰 컴퓨팅 파워와 실행 시간이 필요 합니다.

 따라서, 실시간 서비스가 가능한 수준으로 데이터의 양을 줄이는 작업이 필요합니다. 하지만 다운 샘플링을 하면 특징 정보도 같이 축소 되는 단점이 있어 가능한 특징 정보를 유지 하면서 불필요한 포인트만 제거 하여 데이터 수를 줄이는 방법이 연구 되고 있습니다. 

PCL에서 제공하는 다운 샘플링 기법은 다음과 같습니다.

* 복셀 격자 기반 샘플링
* Uniform 샘플링 



#### 복셀 격자 기반 샘플링

먼저 복셀 의 개념에 대하여 살펴 보겠습니다. 복셀은 2D 이미지를 구성하는 사각형 현태의 최소 단위인 픽셀\(pixel\)을 큐브모양의 3D로 확장한 개념입니다 복셀 자체 는 공간의 좌표를 갖지 않지만 다른 복셀 군과의 위치 관계를 특징 정보로 활용 할수있습니다. 

> 복셀이라는 용어는 부피 \(volume\)와 픽셀 \(pixel\)을 조합한 혼성어 입니다.

\[그림2.8\]는 4x4의 픽셀에 부피 정보를 포함한 4x4x4로 복셀을 도식화 한것입니다.

![&#xADF8;&#xB9BC; 2.8 &#xD53D;&#xC140;&#xACFC; &#xBCF5;&#xC140;](https://user-images.githubusercontent.com/17797922/104988447-6f413400-5a5b-11eb-9de4-4b744a3657ce.png)

복셀화는 point cloud를 Voxel로 변환하는 작업을 의미 합니다. PCL에서는 Voxel Grid filter를 이용하여 복셀화를 진행 합니다. 

진행 방법은 아래와 같습니다. 

1. 사용자 정의로 적합한 Voxel크기\(=leaf\_size\)를 선택 합니다. 
2. 입력 공간을 복셀 단위로 재구성\(나눕니다\) 합니다. 
3. 각 voxel의 leaf size내 포인트\(파란색\) 유무를 계산 합니다. 
4. 복셀내 하나의 점\(빨간색\)을 유지하고 나머지 포인트는 제거 합니다. 

복셀내 하나의 점을 남기는 간단한 방점은 무작위로 하나를 선택 하는 것입니다. 정확성을 높이는 방법은 복셀내 포인트간의 중심점을 계산하는 것입니다 . 복셀 기반 샘플링의 단점은 중심점을 계산하는데 속도 저하가 발생 합니다. Appoximate Voxel Grid방식은 해쉬함수를 이용하여 대략적인 중심점 계산이 가능합니다. 정확도는 다소 떨어지지만 속도 향상을 위해 활용 가능 합니다.

> 중심점 계산은 여러 알고리즘에서 활용 가능합니다. [\[링크\]](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_%28basic%29#Computing_the_centroid)

![&#xADF8;&#xB9BC; 2.9 &#xBCF5;&#xC140;&#xD654; &#xB2E8;&#xACC4;](https://user-images.githubusercontent.com/17797922/104988624-ea0a4f00-5a5b-11eb-876e-389c1f4afaa1.png)

그림 2.9에서는 복셀화가 진행 되는 단계에서 Point 5개가 하나의 포인트로 표현 되었습니다. 즉, 데이터의 크기가 1/5이 되었습니다. voxel 단위\(=leaf\_size\)가 크면 데이터의 양을 더 줄일 수 있습니다. 하지만, 물체 표현력은 줄어 들게 됩니다. 결국 복셀화는 계산 부하와 물체 표현력의 트레이드오프 관계에서 최적의 단위\(=leaf\_size\)를 구하는 것이 가장 중요합니다.

사용되는 파라미터는 아래와 같습니다.

| 파라미터 | 설명 |
| :--- | :--- |
| setInputCloud | 입력 포인트클라우드 |
| setLeafSize |  |
| filter | 결과물 포인트클라우 |

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Downsampling a PointCloud using a VoxelGrid filter
//http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
    pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud); 


  // 읽은 table_scene_lms400.pcd 파일의 포인트수 출력
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // 오브젝트 생성 
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);              //입력
  sor.setLeafSize (0.01f, 0.01f, 0.01f); //leaf size  1cm 
  sor.filter (*cloud_filtered);          //출력 

  // 생성된 포인트클라우드 수 출력 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  // 생성된 포인트클라우드 저장 
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}

```

출력 결과는 아래와 같습니다. 

```cpp
Loaded : 460400 
Result : 455495
```

입력에 사용된 `table_scene_lms400.pcd` 의 전체 포인트 수는 460400개이며 잡음이 제거 되고 455495만 남게 되었습니다. 

시각화를 통해 결과를 확인해 보기 위해서는 아래 명령어를 입력 하면 됩니다.

```cpp
 $ pcl_viewer table_scene_lms400.pcd 
 $ pcl_viewer StatisticalOutlierRemoval.pcd 
 $ pcl_viewer StatisticalOutlierRemoval_Neg.pcd
```



![&#xADF8;&#xB9BC; 2.10 &#xBCF5;&#xC140;&#xD654; &#xACB0;&#xACFC;](https://user-images.githubusercontent.com/17797922/104989179-32763c80-5a5d-11eb-8335-bc56dde309ce.png)



#### Uniform 샘플링 

Uniform sampling은  결과값으로 인덱스\(indices\)를 출력하여 이후 식별자 정보등으로 활용 가능 , ExtractIndices extracts 사용법이 중요 합니다. 

```cpp
// 오프젝트 생성
  pcl::UniformSampling<pcl::PointXYZ> filter ; 
  filter.setInputCloud (cloud) ;     // 입력 
  filter.setRadiusSearch (0.01F) ;   // 탐색 범위 0.01F
  filter.filter (*cloud_filtered) ;  // 필터 적용 
```

### 2.1.3. 관심 영역 설정 

본 챕터에서는 point cloud 필터링 기법을 이용한 관심 영역\(RoI:Region of Interesting\)설정 방법을 살펴 보겠습니다. 관심영역이란 분석을 위해 전체 공간내에서 임의적으로 선택되어진 일부 영역을 말합니다. 일반적으로 사람의 주관적인 결정에 의해서 결정 됩니다. 예를들어 자율 주행차에서 주변 차량을 탐지 할때 도로위를 관심영역으로 설정 합니다. 픽업 로봇의 경우 물건이 있는 박스나 선반을 관심영역으로 설정 합니다. 이렇게 관심 영역외 데이터를 제거 하여 불필요한 연산 부하를 줄일 수 있습니다.

![&#xADF8;&#xB9BC; 2.11 &#xAD00;&#xC2EC; &#xC601;&#xC5ED;](https://user-images.githubusercontent.com/17797922/104989585-1c1cb080-5a5e-11eb-88a0-f5552e36ab64.png)

PCL에서 제공하는 아래 기능으로 관심영역을 설정 할수 있습니다. 

* PassThrough filter : x,y,z의 최소/최대 범위를 지정 
* Conditional filter : x,y,z의 조건문 형태로 범위를 지정

#### PassThrough filter

PassThrough Filter는 입력값으로 관심 영역 x,y,z의 MIN/MAX를 받아 crop하는 방식으로, 직관적이지만 정교한 부분을 제거하지는 못하는 단점이 있습니다. 

동작 과정은 단순 합니다. 

1. 필터에 적용한 제한영역 정보\(x,y,z\)를 입력 받습니다. 
2. 제한 영역 정보외 지역을 삭제 합니다. 

사용되는 파라미터는 아래와 같습니다.

| 파라미터 | 설명 |
| :--- | :--- |
| setInputCloud | 입력 포인트클라우드 |
| setFilterFieldName | 적용할 좌표 축 \(eg. X,Y,Z\) |
| setFilterLimits | 적용할 값 \(최소, 최대 값\) |
| setFilterLimitsNegative | 적용할 값 외 |
| filter | 결과물 포인트클라우 |

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

출력 결과는 아래와 같습니다. 

```cpp
 Loaded :202627 
 Filtered :72823
```

입력에 사용된 `table_scene_lms400.pcd` 의 전체 포인트 수는 460400개이며 잡음이 제거 되고 455495만 남게 되었습니다. 

시각화를 통해 결과를 확인해 보기 위해서는 아래 명령어를 입력 하면 됩니다. 

```cpp
$ pcl_viewer tabletop.pcd 
$ pcl_viewer tabletop_passthrough.pcd
```

![&#xADF8;&#xB9BC; 2.12 PassThrough filter &#xACB0;&#xACFC;](https://user-images.githubusercontent.com/17797922/104989967-f17f2780-5a5e-11eb-8064-1d060f2f48ff.png)

#### Conditional filter 

조건 기반 이상치 제거 방법은 사용자의 경험과 지식에 의존적인 이상치 제거 방법 입니다. 이 방법은 사용자가 지정한 조건과 해당 조건의 임계치를 기준으로 점을 제거 하거나 보존 하는 방법입니다. xx은 위치 정보, 반사도\(intensity\), 색상\(RGB\), 굴곡\(curvature\)등이 될수 있으며 조건은 GT, GE, LT, LE, EQ. 가 있습니다. 이상치 제거에서 사용한 조건 기반 필터링 방식을 이용하여 관심영역을 설정 할수 있습니다. 조건 대상으로 위치 정보를 사용할경우 Passthrough 필터 보다 좀더 유연한 영역 설정이 가능합니다. 

동작 과정은 아래와 같습니다. 

1. 조건 대상자\(??\)를 선택 합니다. 
2. 조건을 선택 합니다. 
3. 임계치 값을 선택합니다. 
4. 각 점 P\_I에 대하여 조건 대상자가 조건을 임계치 한다면 제거 합니다. 

사용되는 파라미터는 아래와 같습니다.

| 파라미터 | 설명 |
| :--- | :--- |
| setInputCloud | 입력 포인트클라우드 |
| setCondition | 조건 설정 |
| setKeepOrganized |  |
| filter | 결과물 포인트클라우 |

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

출력 결과는 아래와 같습니다. 

```cpp
Loaded :  
Result : .. 
```

시각화를 통해 결과를 확인해 보기 위해서는 아래 명령어를 입력 하면 됩니다. 

```cpp
$ pcl_viewer table_scene_lms400.pcd 
$ pcl_viewer StatisticalOutlierRemoval.pcd 
$ pcl_viewer StatisticalOutlierRemoval_Neg.pcd
```



![&#xADF8;&#xB9BC; 2.13 Conditional filter &#xACB0;&#xACFC;](https://user-images.githubusercontent.com/17797922/104990248-94d03c80-5a5f-11eb-9120-07f0c9be729c.png)

> 포인트의 좌표 정보는 Cloudcompare나 ROS/Rviz 등의 시각화 툴을 이용하여 확인 할 수 있습니다.

