# 분류 

> 본 챕터에서 다루는 내용은 PCL과는 거리가 있습니다. PCL을 이용하여 생성된 데이터를 활용하는 방법에 초점

기계 학습의 꽃이자 튜토리얼의 최종 목적은 점군의 분류 입니다. 점군이 입력되면 1장의 전처리 절차를 통해 분석이 가능한 형태로 변환 하게 됩니다. 이후 2장의 챕터 1을 통해서 분석 대상별로 나누게 되면 모든 준비 작업은 끝납니다. 분류를 위해서는 분류의 기준이 되는 특징 정보가 필요 합니다. 사람을 남성/여성으로 분류 할때 사용 할 특징 정보는 머리카락 길이, 근육량이 될것입니다. 

2장의 챕터 3를 통해서 점군의 가장 기본적인 특징인 Normal을 살펴 보았습니다. 이외에도 다양한 특징 정보들이 존재 하며 이를 잘 선택하고, 새로운 특징 정보를 만들어 내는 것이 중요 합니다. 이를 Feature Engineering이라고도 합니다. 

분류를 위해서는 특징 정보외에도 분류 알고리즘이 중요 합니다. 알고리즘은 데이터 속성 및 분류 목적에 따라 선택이 가능합니다. 머신러닝[[^1]](#1) 기반 점군 분류에서는 SVM을 적용 할 수 있습니다. 



본 챕터에서는 3가지 예시를 이용하여 다양한 점군 분류 방법에대하여 살펴 보도록 하겠습니다. 

- 물건 분류(RGBD-SVM)
- 사람 분류(LIDAR-SVM)
- 사람 분류(LIDAR-RandomForest)


|![](https://github.com/camisatx/RoboticsND/raw/master/projects/perception/misc/test_3_object_recognition.png)|![](https://i.imgur.com/9R1smHJ.png)|
|-|-|
|물건 분류|사람 분류|


## 1. 물건 분류(RGBD-SVM)

첫번째 예시에서는 RGB-D센서로 탐지된 물체를 분류 합니다. RGBD센서는 물체의 깊이 정보와 카메라와 같이 색상 정보를 얻을수 있는 장점이 있습니다. 따라서, 활용할 특징 정보도 Normal외 색상 정보를 같이 사용 합니다. 알고리즘으로는 SVM을 적용 하였습니다. 

- 알고리즘 : SVM 
- 특징 정보 
    - Color Histograms
    - Normal Histograms
- Label : Soap, Biscuits, Snacks, ...
- 출처 : [Udacity Robot Nano Degree Perception](https://github.com/hortovanyi/RoboND-Perception-Project/tree/master/output)


## 2. 사람 분류(LIDAR-SVM)


두번째 예시에서는 LIDAR 센서로 탐지된 사람을 분류 합니다. LIDAR센서는 위치(Geometry)정보말고는 다른 정보를 하지 않습니다[[^2]](#2). 따라서, Feature Engineering을 통해서 x,y,z에서 새로운 특징 정보를 추출 하여 사용 합니다. 알고리즘으로는 SVM을 적용 하였습니다. 

- 알고리즘 : SVM 
- 특징 정보 
    - Surface Normal
    - 3D covariance matrix
    - 2D covariance matrix in each zone
    - normalized 2D histogram in main plane
    - normalized 2D histogram in secondary plane
    - Slice feature 
    - 3D box size 
- Label : Human or Not
- 출처 : [Online Learning for Human Classification in 3D LiDAR-based Tracking](http://webpages.lincoln.ac.uk/nbellotto/doc/Yan2017.pdf), IROS2017 

## 3. 사람 분류(LIDAR-Random Forest)

세번째 예시는 두번째 예시와 비슷합니다. 단지 알고리즘으로는 Random Forest을 적용 하였습니다. 


---

<a name="1">[1]</a> 머신러닝과 딥러닝의 분류는 Neural Network기반 방식이냐 아니냐로 구분 하였습니다. 
<a name="2">[2]</a> 일부 Lidar에서는 Intensity 값을 활용하여 분류 작업을 진행 하기도 하지만, 아직은 특정 제품에 한정되어 있어 제외 하였습니다. 
