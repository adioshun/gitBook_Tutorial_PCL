# Normal Estimation 


점군에서 구할수 있는 Feature중에 가장 간단한 Surface Normal에 대하여 살펴 보도록 하겠습니다. 먼저 Normal은 삼차원 공간에서는 공간에 있는 평면 위의 한 점을 지나면서 그 평면에 수직인 직선을 의미합니다. Normal은 크게 꼭지점 법(Vertex Normals)과 평면 법선(Face/surface Normals)로 나누어 집니다. 여기서는 평면 법선만을 다루고 있으며, 줄여서 Normal이라고 표기 하였습니다. 

|![](https://i.imgur.com/eMhFGch.png)|![](https://i.imgur.com/i6n3yEa.png)|
|-|-|
|Normal 종류|3D Surface Normal|

정의 : The normal of a plane is an unit vector that is perpendicular to it

Normal Estimation은 샘플링 된 값들로부터 방향 정보를 복원해 내는 작업을 의미 합니다. 여기서 중요한 것은 샘플링된 값들입니다. 한점의 보만으로는 법선 벡터를 구할수 없습니다. 그래서 구하려고 하는 대상 점의 이웃한 점들이 가지고 있는 값들을 이용하면 샘플링하기 전에 그 점을 포함하고 있던 면의 법선 벡터를 근사적으로 추정 할수 있다. 이렇게 대상점을 중심으로 한 국소적인 정보로 부터 구해 낸 법선 벡터를 추정 법선 벡터(estimated normal)라 합니다. PCL에서는 샘플링 방법으로 이전장에서 살펴본 Octree Search를 사용합니다. 


![image](https://user-images.githubusercontent.com/17797922/41693140-e87b4298-753e-11e8-8d66-0c1ca989e531.png)


> Integral images도 normal estimation방법 중 하나입니다. 하지만 RGB-D센서등에서 얻은 organized clouds를 대상으로 하고 있어 여기서는 다루지 않습니다. 


