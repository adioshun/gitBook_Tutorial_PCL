# RoI Filter


본 챕터에서는 point cloud 필터링 기법 중 관심 영역(RoI:Region of Interesting)을 추출하는 RoI Filtering에 대하여 다루고 있습니다.

PCL에서는 PassThrough filter, Conditional Filter란 이름으로 해당 기능을 제공 하고 있습니다.

- PassThrough Filter는 입력값으로 관심 영역 x,y,z의 최대/최소값을 받아 crop하는 방식으로, 직관적이지만 정교한 부분을 제거하지는 못하는 단점이 있습니다.
- Conditional Filters는 입력값으로 관심 영역 x,y,z의 GT, GE, LT, LE, EQ을 받아 crop하는 방식입니다. 

> PCL-Tutorial에서는 Conditional Filter를 Noise제거용으로 소개 하고 있지만, RoI추출에 좀더 유용한것 같아 RoI 추출 기법으로 분류 하였습니다.

