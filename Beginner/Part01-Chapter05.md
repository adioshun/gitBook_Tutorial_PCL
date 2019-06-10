# 바닥제거 

다음장에서 살펴볼 segmentation을 위해서 바닥제거는 필수적인 절차 입니다. 지면을 제거하게 되면 각각의 오브젝트 들이 서로 연결되지 않고 떨어지기때문에 구분이 쉬워집니다. 

이전 관심영역설정 챕터에서 z값을 이용하여 지표면에서 가까운 point cloud를 바닥으로 인지하고 제거 하는 방법을 간단히 살펴 보았습니다. 하지만 이 방법은 센서 설치에 대한 고려와 경사 등 평면이 아닌 바닥을 제대로 탐지 할 수 없습니다. 

본 챕터에서는 RANSAC의 평면(Plane)모델을 이용하여 바닥을 제거 하는 방법에 대하여 다루고 있습니다. 

## RANSAC이란. 

RANSAC은 `Random Sample Consensus`의 약어로 1981년 Martin A에 의해 제안된 방법입니다[[^1]](#1). 선, 원통, 평면 등과 같은 특정 Model을 알고 있다는 가정하에 특정 cloud point가 이 model에 속하는지 아닌지를 무작위(Random)로 point들을 선별(sample)하여 일치(Consensus)하는지 테스트 하는 방법을 이야기 합니다.  평면모델을 사용하여 바닥제거를 수행 할수 있습니다. 

|![](http://pointclouds.org/documentation/tutorials/_images/random_sample_example1.png)|![](http://pointclouds.org/documentation/tutorials/_images/random_sample_example2.png)|
|-|-|

왼쪽 이미지의 회색은 입력 데이터로 사용되는 Point cloud입니다. 오른쪽 이미지에서 파란색 Line은 사용하고자 하는 Model입니다. RANSAC에서는 모델에 맞으면 Inlier라 하고 맞지 않으면 outlier라고 합니다. 즉, 빨간색 점은 Outlier이고, 파란색 점은 Inlier입니다. 


수행 방법은 무작위로 샘플을 선택(hypothetical inliers라고함) 하여 반복적으로 아래 절차를 진행 한다. 
1. A model is fitted to the hypothetical inliers, i.e. all free parameters of the model are reconstructed from the inliers.
2. All other data are then tested against the fitted model and, if a point fits well to the estimated model, also considered as a hypothetical inlier.
3. The estimated model is reasonably good if sufficiently many points have been classified as hypothetical inliers.
4. The model is reestimated from all hypothetical inliers, because it has only been estimated from the initial set of hypothetical inliers.
5.  Finally, the model is evaluated by estimating the error of the inliers relative to the model.


RANSAC의 장점은 모델 파라미터에 대해 강건한 예측수행이 가능합니. 즉, outliers이 많이 포함되어 있어도 높은 정확도를 보인다.




---

<a name="1">[1]</a> "Random Sample Consensus: A Paradigm for Model Fitting with Application to Image Analysis and Automated Cartography", 1981 by Martin A. Fischler and Robert C. Bolles