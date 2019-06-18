# 용량 줄이기

센서를 통해 수집되는 point cloud는 상황에 따라 데이터양이 많거나, 적을 수 있습니다. 점군 Resampling은 이러한 경우 목적에 따라서 점군의 수를 줄이거나(Donsampling) 늘리는것(Upsampling)을 의미 합니다.

- 점군의 수를 줄이는 것을 다운샘플링이라 하며, 연산 부하 감소 등의 목적으로 수행 합니다. : Voxel Grid

- 점군의 수를 늘리는 것을 업샘플링이라 하며, 탐지/식별 정확도 향상 등을 목적으로 수행 합니다. : surface reconstruction




---

## 다운 샘플링 

#### Voxel이란 


|![](https://i.imgur.com/XuyeCSN.png)|![](https://i.imgur.com/Giq72P9.png)|
|-|-|
|개념|활용 |


복셀은 2D 이미지를 구성하는 최소 단위인 pixel(picture element)을 3D로 확장한것입니다. 즉, 이미지 1x1에서 깊이 정보를 포함한 1x1x1로 표현하고 이때의 최소 단위를 Voxel(Volume + Pixel)이라고 합니다. 단위는 고정되어 있지 않고 사용자가 정의 가능 합니다.




> 일부 문서에서는 3D Box, Cube라고도 표현 합니다. 


#### 복셀화(voxelization) 방법 




복셀화는 point cloud를 Voxel로 변환하는 작업을 의미 합니다. PCL에서는 `Voxel Grid filter`를 이용하여 복셀화를 진행 합니다. 

진행 방법은 아래와 같습니다. 
1. 사용자 정의로 적합한 Voxel크기(=leaf_size)를 선택 합니다. 
2. 각 voxel의 중심점에서 leaf size내 포인트(파란색) 유무를 계산 합니다. 
3. 포인트들의 중심점(빨간색)을 계산 하고 나머지 포인트는 제거 합니다. 

![](https://i.imgur.com/fOvqIqv.png)


위 그림의 예에서는 Point 5개가 하나의 포인트(=Voxel)로 표현 되었습니다. 즉, 데이터의 크기가 1/5이 되었습니다. voxel 단위(=`leaf_size`)가크면 데이터의 양을 더 줄일수 있습니다. 하지만, 물체 표현력은 줄어 들게 됩니다. 결국 복셀화는 계산 부하와 물체 표현력의 트레이드 오프 관계에서 최적의 단위(=`leaf_size`)를 구하는 것이 가장 중요합니다. 

> 예시에서 사용되는 `tabletop.pcd`에서는 실험을 통해 0.01 이 최적임을 도출 하였습니다. 

---

## 업샘플링

### Moving least squares 이란 

https://en.wikipedia.org/wiki/Moving_least_squares