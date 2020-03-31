# 정합 \(70%\)

본 챕터에서 살펴볼 정합은 두개 이상의 점을 하나의 파일로 합치는 방법 중 하나입니다. 이와 같은 상황은 보통 아래와 같은 경우에 발생 합니다.

* 동일한 위치에서 시간차이를 두고 측정된 두개의 점군 `SensorA_t1`과 `SensorA_t2`의 정합. 이 경우 \[Snippets-Concatenating two clouds\]의 방법으로 수행 가능 합니다. 
* 서로다른 위치에서 동일시간 또는 시간차이를 두고 측정된 두개의 점군 `SensorA_tx`와 `SensorB_ty`의 정합. PCL에서는 말하는 정합은 이 경우에 속하며 다양한 정합\(Registration\)알고리즘을 제공 하고 있습니다. 

다시 말해서 정합이란 각 센서를 기준으로 생성된 지역 좌표계 \(local coordinate system\)를 가지는 점군을 하나의 전역 좌표계\(global coordinate system\)로 표현하는 과정을 의미 합니다. 여기서 기준 좌표계는 위도/좌도와 같이 지구상 절대 좌표계 될 수도 있고 두개의 지역좌표계에서 사용자가 지정한 하나를 전역 좌표계로 지정 할 수도 있습니다.

> PCL 문서에서는 Mobile 지역좌표계를 Source, Fixed 전역 좌표계를 Target으로 표현 하고 있습니다.

정합이 제대로 수행되지 않는다면 공간 및 물체에 대한 정확한 모델을 얻기 어려우며, 탐지 분야에서는 하나의 물체 두 개의 물체로 인지하는 중복 탐지 문제가 발생 할 수 있습니다.

정합 알고리즘으로 아래와 같은 것들이 있습니다.

* ICP 

정합 알고리즘의 최종 목적은 전역 좌표계로의 지역 좌표계 이동 및 회전량을 위한 **변환행렬** 추출 입니다. 이렇게 추출된 변환 행렬은 \[Snippets-Using a matrix to transform a point cloud\]을 통해서 변환\(=정합\)을 수행 합니다.

![](https://i.imgur.com/FfTuKxN.png)

## ICP \(Iterative closest points\)

ICP는 이름에서 알수 있듯이 반복적인 수행을 통해서 두 점군의 키포인트의 거리가 가까운 **변환행렬**을 추출 하는것을 목적으로 하고 있습니다.

m\_a개의 점을 포함하는 소스 점군\(source point cloud\) A를 mb개의 점을 포함하는 타갯 점군\(target point cloud\) B에 정합한다고 가정하자. 이 때 ICP 알고리즘은 그림 2처럼 여섯 단계로 이루어져 있으며, 각 단계는 다음과 같이 요약할 수 있다.

![](http://journal.cg-korea.org/journal/jkcgs/jkcgs-24-5/gif/jkcgs-24-5-11-g2.gif)

* 첫째, 점 선택\(point selection\) 단계에서는 알고리즘의 계산량을 줄이기 위하여 노이즈 필터링이나 샘플링을 거쳐 점의 갯수를 줄인다.
* 둘째, 이웃 선택\(neighborhood selection\) 단계에서는 각 점의 주변에 분포하는 점들 중 가까운 점 일부를 선택한다.
* 셋째, 점쌍 매칭\(point pair matching\) 단계에서는 앞서 선택한 주변 점들로 곡면\(surface\)이나 평면\(plane\) 같은 기하학 요소를 만들고, 이를 이용하여 소스 점군 A의 점 에 대응하는\(corresponding\) 타갯 점군 표의 점을 찾는다.
* 넷째, 오정합점 제거 \(outlier rejection\) 단계에서는 소스 점군 A와 타겟 점군 B에서 서로 겹치는 부분, 즉 오버랩 영역\(inlier\)만 남기고, 겹치지 않는 영역\(outlier\) 은 정합에 이용하지 않는다.
* 다섯째, 오차 최소화\(error minimization\) 단계 에서는 소스 점군 A의 오버랩 영역 A′과 타겟 점군 B의 오버랩 영역 B′ 방향으로 이동하는 동안 각 대응점의 거리 오차가 최소가 되도록 한다.
* 여섯째, 변환\(Transformation\) 단계에서 는 소스 오버랩 점군 A′가 타갯 오버랩 점군 B′의 위치로 이동하 는 데에 필요한병진 행렬\(translation matrix\)과 회전 행렬\(rotation matrix\)을 구하여 소스 오버랩 점군 A′에 적용한다.
  * 여기서 거리 오차가 일정한 한계치\(tolerance, τ\) 이하이면 진행을 멈주고 한계치 이상이라면 점쌍 매칭 \(point pair matching\) 단계로 되돌아가서 알고리즘을 다시 수행한다.

[\[1\]](./) 김지건, "[적은 오버랩에서 사용 가능한 3차원 점군 정합 방법](http://journal.cg-korea.org/archive/view_article?pid=jkcgs-24-5-11)", 2018

