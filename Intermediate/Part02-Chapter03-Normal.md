# Normal Estimation 


점군에서 구할수 있는 Feature중에 가장 간단한 Surface Normal에 대하여 살펴 보도록 하겠습니다. 먼저 Normal은 삼차원 공간에서는 공간에 있는 평면 위의 한 점을 지나면서 그 평면에 수직인 직선을 의미합니다. Normal은 크게 꼭지점 법(Vertex Normals)과 평면 법선(Face/surface Normals)로 나누어 집니다. 여기서는 평면 법선만을 다루며 줄여서 Normal 이라고 표기 하였습니다. 

|![](https://i.imgur.com/eMhFGch.png)|![](https://i.imgur.com/i6n3yEa.png)|
|-|-|
|Normal 종류|3D Surface Normal|

정의 : The normal of a plane is an unit vector that is perpendicular to it






---

Integral images
Integral images are a method for normal estimation on organized clouds. The algorithm sees the cloud as a depth image, and creates certain rectangular areas over which the normals are computed, by taking into account the relationship between neighboring "pixels" (points), without the need to run lookups on a search structure like a tree. Because of this, it is very efficient and the normals are usually computed in a split second. If you are performing real time computations on clouds taken from a RGB-D sensor, this is the approach you should use because the previous method will probably take several seconds to finish.