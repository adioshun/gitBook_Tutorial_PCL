# INTRO

## 개요

Point cloud Library\(PCL\)는 LIDAR나 RGB-D센서의 3D 데이터 처리를 위해 필수적인 툴 중 하나입니다. 하지만 개발자 홈페이지를 제외 하고는 정리 되어 있는 문서나 질문/의견을 교류 할 수 있는 곳이 적은 것 같습니다.

그동안 익혔던 내용 및 코드도 정리하고 다른분들에게 조금이라도 도움이 되고자 Tutorial형식으로 작성 하게 되었습니다.

* 기본 내용은 PCL 홈페이지의 [\[PCL문서\]](http://pointclouds.org/documentation/tutorials/)와 [\[Udacity강좌\]](https://github.com/udacity/RoboND-Perception-Exercises)를 중심으로 하였습니다.
* 그 외 참고한 여러 자료들은 [_References_](appendix/references.md)페이지에 별도 업데이트 하도록 하겠습니다.
* PCL에 대한 정보 공유나 궁금한점은 [\[페이스북 PCL Research Group KR\]](https://www.facebook.com/groups/165198587522918/)에 올려 주세요.
* Tutorial은 [초안\(Gitbook\)](https://adioshun.gitbooks.io/pcl-tutorial/content/), [백업\(Github\)](https://github.com/adioshun/gitBook_Tutorial_PCL), [최종본](https://pcl.gitbook.io/tutorial)에 동시 저장되어 있습니다.

## 문서 구성 및 내용

전체 문서는 초급/중급/고급으로 나누어져\(Part 1~3\) 있으며, 각 문서에는 이론, 각 Library별 코드, 실습으로 구성 되어 있습니다.

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

