# README

본 장에서는 PCL에서 제공하는 기능들을 기반으로 3D 데이터 전처리, 필터링를 다루고 있습니다.

* Chapter 1 : 3D 데이터 읽고 저장하기 : PCD Read and Save 
* Chapter 2 : 작업 부하를 줄이기 위한 Downsampling : VoxelGrid filter
* Chapter 3 : 대상 영역 선정을 위한 RoI 설정 : PassThrough filter
* Chapter 4 : 센서 잡음 제거를 위한 Outlier제거 : Statistical/Conditional/Radius Outlier removal
* Chapter 5 : 바 세그멘테이션 및 제거 : RANSAC 

또한, 이해 돕기를 위해 각 챕터의 결과 확인에 맞는 PCD파일들을 사용하고 있습니다.

| ![image](https://user-images.githubusercontent.com/17797922/41080489-9d804f18-69db-11e8-8a8b-9422c2e13132.png) | ![](https://i.imgur.com/pdSfhsW.png) |
| :--- | :--- |
| [Udacity Sample \(`table.top`\)](https://github.com/udacity/RoboND-Perception-Exercises/raw/master/Exercise-1/tabletop.pcd) | [PCL Tutorial sample\(`table_scene_lms400.pcd`\)](https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd%20) |
| Chapter 3,5 | Chapter 2,4 |

* [파일 생성 및 입출력 \(70%\)](part01-chapter01/)
  * [PCL-Cpp \(70%\)](part01-chapter01/part01-chapter01-pcl-cpp.md)
  * [PCL-Python \(70%\)](part01-chapter01/part01-chapter01-pcl-python.md)
  * [Open3D-Python \(70%\)](part01-chapter01/part01-chapter01-open3d-python.md)
  * [ROS 실습 \(90%\)](part01-chapter01/part01-chapter01-practice.md)
* [샘플링 \(70%\)](part01-chapter02/)
  * [다운샘플링-PCL-Cpp \(70%\)](part01-chapter02/part01-chapter02-downsampling-pcl-cpp.md)
  * [다운샘플링-PCL-Python \(50%\)](part01-chapter02/part01-chapter02-downsampling-pcl-python.md)
  * [업샘플링-PCL-Cpp \(70%\)](part01-chapter02/part01-chapter02-upsampling-pcl-cpp.md)
  * [ROS 실습 \(90%\)](part01-chapter02/part01-chapter02-practice.md)
* [관심 영역 설정 \(70%\)](part01-chapter03/)
  * [PCL-Cpp \(70%\)](part01-chapter03/part01-chapter03-pcl-cpp.md)
  * [PCL-Python \(70%\)](part01-chapter03/part01-chapter03-pcl-python.md)
  * [ROS 실습 \(90%\)](part01-chapter03/part01-chapter03-practice.md)
* [노이즈 제거 \(70%\)](part01-chapter04/)
  * [PCL-Cpp \(70%\)](part01-chapter04/part01-chapter04-pcl-cpp.md)
  * [PCL-Python \(50%\)](part01-chapter04/part01-chapter04-pcl-python.md)
  * [ROS 실습 \(90%\)](part01-chapter04/part01-chapter04-practice.md)
* [바닥제거 \(RANSAC\) \(70%\)](../part-2/part01-chapter05/)
  * [PCL-Cpp \(70%\)](../part-2/part01-chapter05/part01-chapter05-pcl-cpp.md)
  * [PCL-Python \(70%\)](../part-2/part01-chapter05/part01-chapter05-pcl-python.md)
  * [ROS 실습 \(90%\)](../part-2/part01-chapter05/part01-chapter05-practice.md)

