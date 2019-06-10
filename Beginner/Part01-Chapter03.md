# 관심 영역 설정 


본 챕터에서는 point cloud 필터링 기법 중 관심 영역(RoI:Region of Interesting)을 추출하는 RoI Filtering에 대하여 다루고 있습니다.

관심 영역 설정은 이전 챕터의 Downsampling과 같이 점군을 줄여 불필요한 연산 부하를 줄일때 사용됩니다. 

![](https://lh4.googleusercontent.com/lKmtfSR8WLigeGnRySnHr85XPH-cIbmJk1QPxN2WycvMbThPFNK63Yr1qKVCEoKeK7DwjBbiYLjJ_LpZgJhbWik6N4U2rJlYWgT7sHkbJ1uxWiY2BcPo4rry8DFEhvcZIkmTbod6LctqSF8bnw)

예를들어 불법 주차를 단속 하는 프로그램을 작성할때 단속 공간(녹색박스)내 차량의 유무만 판단하면 되기 때문에 나머지 영역을 감시할 필요가 없습니다. 이렇게 관심 영역외 지역을 삭제 함을써 연산 부하를 줄일수 있습니다. 이때 필요한 정보는 공간상 녹색 박스의 위치 정보 입니다. 

PCL에서는 PassThrough filter, Conditional Filter[[^1]](#1)란 이름으로 해당 기능을 제공 하고 있습니다.

- PassThrough Filter는 입력값으로 관심 영역 x,y,z의 MIN/MAX를 받아 crop하는 방식으로, 직관적이지만 정교한 부분을 제거하지는 못하는 단점이 있습니다.
- Conditional Filters는 입력값으로 관심 영역 x,y,z의 GT, GE, LT, LE, EQ을 받아 crop하는 방식입니다. 

먼저 관심 영역 추출은 센서의 위치가 고정되어 있다는 가정하에 진행 됩니다. 예를 들어 공장의 선반 로봇처럼 센서는 고정되어 있고, 로봇의 팔만 움직이는 경우에는 위 RoI필터 적용이 가능합니다. 

동작 과정은 단순 합니다. 
1. 필터에 적용한 제한영역 정보를 입력 받습니다. 
2. 제한 영역 정보외 지역을 삭제 합니다. 


## ROI Filter 적용 

|![](https://i.imgur.com/ESXyPtq.png)|![](https://i.imgur.com/oSzk5Kb.png)|![](https://i.imgur.com/7jbrH33.png)|
|-|-|-|
|원본|관심영역 설정 | 관심영역 필터링 결과(x,y)|

위 그림은 어느 건물의 내부 포인트 클라우드 데이터입니다. 중앙 Hall부분(사각형)의 사람만 탐지 하고자 할경우 x는 1.0~20.0, y는 -7.0~5.5영역을  설정한것입니다. 

|![](https://i.imgur.com/OFoOiVA.png) |![](https://i.imgur.com/Oxt0KJr.png)|
|-|-|
|원본|관심영역 필터링 결과(z)

위 그림은 바닥제거를 위해 ROI Filter를 사용한 것입니다. y좌표 -2.0 부분에 타원형 선들은 센서가 바닥을 탐지 하여 표시한 것입니다. 바닥영역의 데이터들은 불필요 할뿐만 아니라 추후 군집화 알고리즘 등을 활용시 정확도 성능에 영향을 주므로 삭제 하는것이 좋습니다. 위 예시에서는 z는 -1.2~10.0으로 설정한것입니다. 

하지만 두개의 선이 남아 있는것을 볼수 있습니다. 센서 설치 단계에서 기울어져있으면 근거리와 원거리의 z값이 다르기 때문에 센서 설치시 수평을 맞추는 것이 중요 합니다. 

> ROI필터를 이용한 바닥제거는 설명을 위해 포함된 것으로 뒤에 RANSAC이용한 방법이 좀더 강건하게 바닥을 탐지 할수 있습니다. 

---


<a name="1">[1]</a>  PCL-Tutorial에서는 Conditional Filter를 Noise제거용으로 소개 하고 있지만, RoI추출에 좀더 유용한것 같아 RoI 추출 기법으로 분류 하였습니다.

