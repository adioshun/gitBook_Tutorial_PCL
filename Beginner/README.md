# 1장 개요 


본 장에서는 PCL에서 제공하는 기능들을 기반으로 3D 데이터 전처리, 필터링를 다루고 있습니다. 

- Chapter 1 : 3D 데이터 읽고 저장하기 : PCD Read and Save 
- Chapter 2 : 작업 부하를 줄이기 위한 Downsampling : VoxelGrid filter
- Chapter 3 : 대상 영역 선정을 위한 RoI 설정 : PassThrough filter
- Chapter 4 : 센서 잡음 제거를 위한 Outlier제거 : Statistical/Conditional/Radius Outlier removal
- Chapter 5 : 바 세그멘테이션 및 제거 : RANSAC 





또한, 이해 돕기를 위해 각 챕터의 결과 확인에 맞는 PCD파일들을 사용하고 있습니다. 

|![image](https://user-images.githubusercontent.com/17797922/41080489-9d804f18-69db-11e8-8a8b-9422c2e13132.png)|![](https://i.imgur.com/pdSfhsW.png)|
|-|-|
|[Udacity Sample (`table.top`)](https://github.com/udacity/RoboND-Perception-Exercises/raw/master/Exercise-1/tabletop.pcd)|[PCL Tutorial sample(`table_scene_lms400.pcd`)](https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd ) |
|Chapter 3,5|Chapter 2,4|

---
* [파일 생성 및 입출력 \(70%\)](Part01-Chapter01.md)
    * [PCL-Cpp \(70%\)](Part01-Chapter01-PCL-Cpp.md)
    * [PCL-Python \(70%\)](Part01-Chapter01-PCL-Python.md)
    * [Open3D-Python \(70%\)](Part01-Chapter01-Open3D-Python.md)
    * [ROS 실습 \(90%\)](Part01-Chapter01-Practice.md)
* [샘플링 \(70%\)](Part01-Chapter02.md)
    * [다운샘플링-PCL-Cpp \(70%\)](Part01-Chapter02-Downsampling-PCL-Cpp.md)
    * [다운샘플링-PCL-Python \(50%\)](Part01-Chapter02-Downsampling-PCL-Python.md)
    * [업샘플링-PCL-Cpp \(70%\)](Part01-Chapter02-Upsampling-PCL-Cpp.md)
    * [ROS 실습 \(90%\)](Part01-Chapter02-Practice.md)
* [관심 영역 설정 \(70%\)](Part01-Chapter03.md)
    * [PCL-Cpp \(70%\)](Part01-Chapter03-PCL-Cpp.md)
    * [PCL-Python \(70%\)](Part01-Chapter03-PCL-Python.md)
    * [ROS 실습 \(90%\)](Part01-Chapter03-Practice.md)
* [노이즈 제거 \(70%\)](Part01-Chapter04.md)
    * [PCL-Cpp \(70%\)](Part01-Chapter04-PCL-Cpp.md)
    * [PCL-Python \(50%\)](Part01-Chapter04-PCL-Python.md)
    * [ROS 실습 \(90%\)](Part01-Chapter04-Practice.md)
* [바닥제거 \(RANSAC\) \(70%\)](Part01-Chapter05.md)
    * [PCL-Cpp \(70%\)](Part01-Chapter05-PCL-Cpp.md)
    * [PCL-Python \(70%\)](Part01-Chapter05-PCL-Python.md)
    * [ROS 실습 \(90%\)](Part01-Chapter05-Practice.md)
