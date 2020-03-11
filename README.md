# 개요 

Point cloud Library(PCL)는 LIDAR나 RGB-D센서의 3D 데이터 처리를 위해 필수적인 툴 중 하나입니다. 하지만 개발자 홈페이지를 제외 하고는 정리 되어 있는 문서나 질문/의견을 교류 할 수 있는 곳이 적은 것 같습니다. 

그동안 익혔던 내용 및 코드도 정리하고 다른분들에게 조금이라도 도움이 되고자 Tutorial형식으로 작성 하게 되었습니다. 

- 기본 내용은 PCL 홈페이지의 [[PCL문서]](http://pointclouds.org/documentation/tutorials/)와 [[Udacity강좌]](https://github.com/udacity/RoboND-Perception-Exercises)를 중심으로 하였습니다. 

- 그 외 참고한 여러 자료들은 *[References](references.md)*페이지에 별도 업데이트 하도록 하겠습니다. 

- PCL에 대한 정보 공유나 궁금한점은 [[페이스북 PCL Research Group KR]](https://www.facebook.com/groups/165198587522918/)에 올려 주세요. 

- Tutorial은 [초안(Gitbook)](https://adioshun.gitbooks.io/pcl-tutorial/content/), [백업(Github)](https://github.com/adioshun/gitBook_Tutorial_PCL), [최종본](https://pcl.gitbook.io/tutorial)에 동시 저장되어 있습니다. 


# 문서 구성 및 내용 

전체 문서는 초급/중급/고급으로 나누어져(Part 1~3) 있으며, 각 문서에는 이론, 각 Library별 코드, 실습으로 구성 되어 있습니다. 


### 이론 

초급에서는 PCL에서 제공하는 기능들을 기반으로 3D 데이터 전처리, 필터링를 다루고 있습니다. 

중급에서는 PCL에서 제공하는 기능들을 기반으로 Clustering, Classification, Octree, Registration를 다루고 있습니다. 

고급에서는 최근 트랜드인 딥러닝을 이용하여 초급/중급에서 살펴본 샘플링, Classification 등을 Deep Neural Network를 이용하여 구현해 보려 합니다. 




### 각 Library별 코드

각 이론별 코드에서는 PCL C++, PCL-Python, Open3D-Python별 샘플 코드를 포함하고 있습니다. 




### 실습 

실습은 ROS를 기반으로 진행 됩니다. 

실습 초/중급에서는 Velodyne Puck 16ch 라이다를 이용하여 실제로 수집한 데이터에서 위 이론내용과 코드들을 활용하여 재실자 탐지를 진행 합니다. 

실습 고급에서는 KITTI 및 오픈데이터셋을 이용 이용하여 차량 탐지를 목적으로 진행 합니다. 

---
## Part 0 \(개요\)

* [README](Introduction/README.md)
* [chapter01 : PCL & PCD란 \(100%\)](Introduction/Part00-Chapter01.md)
* [chapter02 : PCL 설치 \(100%\)](Introduction/Part00-Chapter02.md)
* [chapter03 : ROS 실습 준비\(100%\)](Introduction/Part00-Chapter03.md)

## Part 1 \(초급\)

* [README](Beginner/README.md)
* [파일 생성 및 입출력 \(70%\)](Beginner/Part01-Chapter01.md)
  * [PCL-Cpp \(70%\)](Beginner/Part01-Chapter01-PCL-Cpp.md)
  * [PCL-Python \(70%\)](Beginner/Part01-Chapter01-PCL-Python.md)
  * [Open3D-Python \(70%\)](Beginner/Part01-Chapter01-Open3D-Python.md)
  * [ROS 실습 \(90%\)](Beginner/Part01-Chapter01-Practice.md)
* [샘플링 \(70%\)](Beginner/Part01-Chapter02.md)
  * [다운샘플링-PCL-Cpp \(70%\)](Beginner/Part01-Chapter02-Downsampling-PCL-Cpp.md)
  * [다운샘플링-PCL-Python  \(50%\)](Beginner/Part01-Chapter02-Downsampling-PCL-Python.md)
  * [업샘플링-PCL-Cpp  \(70%\)](Beginner/Part01-Chapter02-Upsampling-PCL-Cpp.md)
  * [ROS 실습 \(90%\)](Beginner/Part01-Chapter02-Practice.md)
* [관심 영역 설정  \(70%\)](Beginner/Part01-Chapter03.md)
  * [PCL-Cpp  \(70%\)](Beginner/Part01-Chapter03-PCL-Cpp.md)
  * [PCL-Python  \(70%\)](Beginner/Part01-Chapter03-PCL-Python.md)
  * [ROS 실습 \(90%\)](Beginner/Part01-Chapter03-Practice.md)
* [노이즈 제거  \(70%\)](Beginner/Part01-Chapter04.md)
  * [PCL-Cpp  \(70%\)](Beginner/Part01-Chapter04-PCL-Cpp.md)
  * [PCL-Python  \(50%\)](Beginner/Part01-Chapter04-PCL-Python.md)
  * [ROS 실습 \(90%\)](Beginner/Part01-Chapter04-Practice.md)
* [바닥제거 \(RANSAC\)  \(70%\)](Beginner/Part01-Chapter05.md)
  * [PCL-Cpp  \(70%\)](Beginner/Part01-Chapter05-PCL-Cpp.md)
  * [PCL-Python  \(70%\)](Beginner/Part01-Chapter05-PCL-Python.md)
  * [ROS 실습 \(90%\)](Beginner/Part01-Chapter05-Practice.md)

## Part 2 \(중급\)

* [README](Intermediate/README.md)
* [군집화  \(70%\)](Intermediate/Part02-Chapter01.md)
  * [Euclidean-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter01-Euclidean-PCL-Cpp.md)
  * [Euclidean-PCL-Python \(0%\)](Intermediate/Part02-Chapter01-Euclidean-PCL-Python.md)
  * [DBSCAN-PCL-Python \(0%\)](Intermediate/Part02-Chapter01-DBSCAN-PCL-Python.md)
* [포인트 탐색과 배경제거 \(60%\)](Intermediate/Part02-Chapter02.md)
  * [Search-Octree-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter02-Search-Octree-PCL-Cpp.md)
  * [Search-Octree-PCL-Python  \(70%\)](Intermediate/Part02-Chapter02-Search-Octree-PCL-Python.md)
  * [Search-Kdtree-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter02-Search-Kdtree-PCL-Cpp.md)
  * [Search-Kdtree-PCL-Python  \(70%\)](Intermediate/Part02-Chapter02-Search-Kdtree-PCL-Python.md)
  * [Compression-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter02-Compression-PCL-Cpp.md)
  * [DetectChanges-PCL-Cpp  \(50%\)](Intermediate/Part02-Chapter02-DetectChanges-PCL-Cpp.md)
  * [DetectChanges-PCL-Python \(50%\)](Intermediate/Part02-Chapter02-DetectChanges-PCL-Python.md)
* [특징 찾기 \(50%\)](Intermediate/Part02-Chapter03.md)
  * [Normal-PCL-Cpp \(70%\)](Intermediate/Part02-Chapter03-Normal-PCL-Cpp.md)
  * [Normal-PCL-Python \(80%\)](Intermediate/Part02-Chapter03-Normal-PCL-Python.md)
* [분류 \(30%\)](Intermediate/Part02-Chapter04.md)
  * [SVM-RGBD-PCL-Python  \(70%\)](Intermediate/Part02-Chapter04-SVM-RGBD-PCL-Python.md)
  * [SVM-LIDAR-PCL-Python \(0%\)](Intermediate/Part02-Chapter04-SVM-LIDAR-PCL-Python.md)
  * [SVM-ROS \(0%\)](Intermediate/Part02-Chapter04-SVM-ROS.md)
* [정합 \(70%\)](Intermediate/Part02-Chapter05.md)
  * [ICP-PCL-Cpp \(70%\)](Intermediate/Part02-Chapter05-ICP-PCL-Cpp.md)
  * [ICP-ROS 실습 \(10%\)](Intermediate/Part02-Chapter05-ICP-Practice.md)
* [재구성 \(30%\)](Intermediate/Part02-Chapter06.md)
  * [Smoothig-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter06-Smoothig-PCL-Cpp.md)
  * [Smoothig-PCL-Python  \(70%\)](Intermediate/Part02-Chapter06-Smoothig-PCL-Python.md)
  * [Triangulation-PCL-Cpp  \(70%\)](Intermediate/Part02-Chapter06-Triangulation-PCL-Cpp.md)

## Part 3 \(고급\)

* [README](Advanced/README.md)
* [딥러닝 기반 학습 데이터 생성 \(0%\)](Advanced/Part03-Chapter01.md)
  * [PointGAN \(90%\)](Advanced/Part03-Chapter01-PointGAN.md)
  * [AutoEncoder \(0%\)](Advanced/Part03-Chapter01-AutoEncoder.md)
* [딥러닝 기반 샘플링 기법 \(0%\)](Advanced/Part03-Chapter02.md)
  * [DenseLidarNet \(50%\)](Advanced/Part03-Chapter02-DenseLidarNet.md)
  * [Point Cloud Upsampling Network](Advanced/Part03-Chapter02-PUNet.md)
  * [Pseudo-LiDAR](Advanced/Part03-Chapter02-Pseudo-LiDAR.md)
* [딥러닝 기반 자율주행 탐지 기술 \(0%\)](Advanced/Part03-Chapter03.md)
* [딥러닝 기반 자율주행 분류 기술 \(0%\)](Advanced/Part03-Chapter04.md)
  * [Multi3D](Advanced/Part03-Chapter04-Multi3D.md)
  * [PointNet](Advanced/Part03-Chapter04-PointNet.md)
  * [VoxelNet \(50%\)](Advanced/Part03-Chapter04-VoxelNet.md)
  * [YOLO3D](Advanced/Part03-Chapter04-YOLO3D.md)
  * [SqueezeSeg](Advanced/Part03-Chapter04-SqueezeSeg.md)
  * [butNet](Advanced/Part03-Chapter04-butNet.md)

## Snippets

* [PCL-Snippets](https://adioshun.gitbooks.io/pcl_snippet/content/)
* [PCL-Python-Helper \(10%\)](Snippets/PCL-Python-helper.md)
* [Lidar Data Augmentation](Snippets/Lidar-Data-Augmentation.md)

## Appendix

* [시각화Code](Appendix/Visualization.md)
* [시각화툴](Appendix/Visualization-Tools.md)
* [Annotation툴](Appendix/Annotation-Tools.md)
* [Point Cloud Libraries \(0%\)](Appendix/Libraries.md)
* [데이터셋](Appendix/Datasets.md)
* [참고 자료](references.md)

---

* [작성 계획\_Tips](Plan.md)
* [용어집](Terms.md)





