# 노이즈 제거 

모든 센서는 특성상 측정시 에러가 존재 합니다. Lidar의 경우 탐지 물체가 존재 하지 않아도 먼지나 오류 등으로 인해 point가 생성 됩니다. 이렇게 생성된 point들은 Noise또는 Outlier라 하고 제거 하는 작업을 진행 해야 합니다. 이를 Noise filtering또는 Outlier Removal이라고 합니다. 

다행히 이러한 노이즈들은 정상적인 point 대비 고유의 특징을 가지고 있어 이를 활용하여 제거 할수 있습니다.

- Statistical based : 통계적 방법 활용 

- Radius based : 거리 정보 활용 


> **[중요]** 현재 Radius based 방식은 정상 동작 하지 않는다고 합니다. 파라미터를 바꾸어도 결과가 '0'이라고 하네요. [[참고]](https://github.com/strawlab/python-pcl/issues/211) - 2018.06.11


## StatisticalOutlierRemoval filter


통계학적 정보를 이용하여 Noise를 탐지 하는 방법 입니다. 

동작 과정
- 이웃 근접 point 들과의 평균 거리 정보를 계산 합니다. 
- 이 분포가 Gaussian distribution따른다는 가정하에 나머지는 잡음으로 간주 제거 합니다. 

## Radius Outlier removal

거리 정보를 이용한 가장 간단한 Noise를 탐지 하는 방법 입니다. 

동작 과정 
- 탐색 반경 / 최소 포인트 수 정의 
- 특정 포인트의 반경내 지정된 포인트 이하인경우 잡음으로 간주 제거 합니다. 




