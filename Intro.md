# 개요 


새로운 프로젝트를 진행 하면서 Point cloud Library(PCL)를 처음 접하게 되었습니다. 3D 데이터의 처리를 위해서는 필수적인 Library인데 개발자 홈페이지를 제외 하고는 정리 되어 있는 문서나 질문/의견을 교류 할 수 있는 곳 적어 개념을 이해하거나 적용 하는데 많은 어려움이 있었습니다. 

프로젝트 1년차가 종료 되어 가는 시점에서 그동안 익혔던 내용 및 코드를 정리하고, 이 분야를 시작하시려는 분들에게 조금이나마 도움이 되고자 Tutorial을 작성 하게 되었습니다. 


- 기본 내용은 PCL 홈페이지의 [[Tutorials]](http://pointclouds.org/documentation/tutorials/)와 Eugen Cicvarić의 [[3D Object Recognition and Pose Estimation using Point Cloud Library]](https://drive.google.com/file/d/1QtQTlm3_FiOdBslbtMAubVMyd2Bjofl1/view?fbclid=IwAR0NZfTAvfSwg_X_Flx5Uhg5GMLRaNFdgKU6PZRsHuskc95Sd2ErAKLg4LM), [[wikipedia]](https://www.wikipedia.org/)를 중심으로 하였습니다. 

- 그 외 참고한 여러 자료들은 *[References]*페이지에 별도 업데이트 하도록 하겠습니다. 

- PCL에 대한 정보 공유나 궁금한점은 **[[페이스북 PCL Research Group KR]](https://www.facebook.com/groups/165198587522918/)**에 올려 주세요. 

- Tutorial은 [초안(Gitbook)](https://adioshun.gitbooks.io/pcl-tutorial/content/), [백업(Github)](https://github.com/adioshun/gitBook_Tutorial_PCL), [최종본(Wikidocs)](https://wikidocs.net/book/827)에 동시 저장되어 있습니다. 


# 문서 구성 및 내용 

전체 문서는 초급/중급/고급으로 나누어져(Part 1~3) 있으며, 각 문서에는 이론, 각 Library별 코드, 실습으로 구성 되어 있습니다. 


### 이론 

초급에서는 PCL에서 제공하는 기능들을 기반으로 3D 데이터 전처리, 필터링를 다루고 있습니다. 

중급에서는 PCL에서 제공하는 기능들을 기반으로 Clustering, Classification, Octree, Registration를 다루고 있습니다. 

고급에서는 PCL 딥러닝을 연계 하여 학습 데이터 생성등에 쓰이는 PointGAN, 분류에 쓰이는 SECOND 등을 다루고 있습니다. 

각 이론별 코드에서는 PCL C++, PCL-Python, Open3D-Python별 샘플 코드를 포함하고 있습니다. 


또한, 이해 돕기를 위해 여러 공개 PCD파일들을 사용하고 있습니다. 

|![image](https://user-images.githubusercontent.com/17797922/41080489-9d804f18-69db-11e8-8a8b-9422c2e13132.png)|![](https://i.imgur.com/pdSfhsW.png)|
|-|-|
|[RoboND-Perception-Exercises](https://github.com/udacity/RoboND-Perception-Exercises/raw/master/Exercise-1/tabletop.pcd)|[PCL Tutorial sample](https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd ) |






좀더 자세한 구성은 아래와 같습니다. 

```
초급 
- 3D 데이터 읽고 저장하기 : PCD Read and Save 
- 작업 부하를 줄이기 위한 Downsampling : VoxelGrid filter
- 대상 영역 선정을 위한 RoI 설정 : PassThrough filter
- 센서 잡음 제거를 위한 Outlier제거 : Statistical/Conditional/Radius Outlier removal
- 바닦 세그멘테이션 및 제거 : RANSAC 


중급

- Octree Search
- 군집화 : DBSCAN, seed Clustering 
- Feature : Normal 
- 두 좌표계 통합 
- Octree compress
- 포인트 합치기 : Concatenate the points of two Point Clouds
- 학습/분류 : SVM 

고급 
- Range image를 이용한 군집화 
- BEV를 이용한 군집화 
- PointGAN을 이용한 학습 데이터 생성 
- VoxelNET을 이용한 분류 작업 
```

### 실습 

실습은 ROS를 기반으로 진행 됩니다. 

실습 초/중급에서는 Velodyne Puck 16ch 라이다를 이용하여 실제로 수집한 데이터에서 위 이론내용과 코드들을 활용하여 재실자 탐지를 진행 합니다. 

실습 고급에서는 KITTI 학습 데이터를 이용하여 차량 탐지를 진행 합니다. 




---


# 작성 계획 

- 초급 : 2019. 06월 15일 까지 
- 중급 : 2019. 07월 30일 까지 
- 고급 : 2019. 08월 30일 까지 


