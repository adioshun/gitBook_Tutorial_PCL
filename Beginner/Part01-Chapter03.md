# RoI Filter


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


---


<a name="1">[1]</a>  PCL-Tutorial에서는 Conditional Filter를 Noise제거용으로 소개 하고 있지만, RoI추출에 좀더 유용한것 같아 RoI 추출 기법으로 분류 하였습니다.

