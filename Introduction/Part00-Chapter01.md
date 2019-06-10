## Point cloud Library이란? 

PCL은 Lidar 센나 RGB-D센서, Stereo camera 등으로 수집되는 점군(Point cloud)를 처리 하기 위한 라이브러리 입니다. 여기서 점군이란 3차원 공간상에 퍼져 있는 여러 점(Point)의 집합(set=cloud)을 의미 합니다. 

|![](https://i.imgur.com/8kEIXdA.png)|![](https://i.imgur.com/fQrysOa.png)|![](https://c1.staticflickr.com/6/5310/5880903905_ed581013b0.jpg)|
|-|-|-|
|Lidar를 이용한 컵의 Point cloud|RGB-D센서를 이용한 Point Cloud|Stereo Camera를 이용한 Point Cloud|

이러한 점군은 이미지와 다르게 깊이(z)정보를 가지고 있기 때문에 기본적으로 (x,y,z)`N x 3` Numpy 배열로 표현 됩니다. 여기서 각 N 줄은 하나의 점과 맵핑이 됩니다. 

Point Cloud는 기본적으로는 x,y,z로 표현 되지만 센서에서 제공되는 추가 정보가 있을경우 `N x 4`Numpy 배열로도 표현 가능 합니다. 예를 들어 Lidar 센서에서 수집한 정보의 경우는 `reflectance(반사도)`라는 정보가 추가될수 있으며, RGB-D에는 Color정보가 추가 될수 있습니다. 따라서 위에서 살펴볼 Point cloud data format에서는 여러 종류가 있습니다. 

```
참고로 일부 문서에서는 unordered Point cloud라고 표기 하는 경우가 있습니다. 이는 `N x 3` Numpy 배열에서 N의 순서(order)가 물체를 표현하는데 영향을 미치지 않기 때문입니다. 
```

|The Point Cloud Data|이미지 데이터와 비교 |
|-|-|
|![](http://i.imgur.com/Bc13san.png)|![](http://i.imgur.com/smzFU5N.png)|

```
이미지 데이터
- 항상 양수 이다. 
- 기준점은 왼쪽 위부터 이다. 
- 좌표값은 정수(integer)이다. 

포인트 클라우드 데이터 
- 양수/음수 이다. 
- 좌표값은 real numbered이다. 
- The positive x axis represents forward.
- The positive y axis represents left.
- The positive z axis represents up.
```



## PCL종류 


Point cloud를 처리를 위한 라이브러리로는 PCL[[^1]](#1), PCL-python, Open3D, pyPCD, Laspy, PCLpy 등이 있습니다. 


일반적으로 PCL하면 2011년 Radu Bogdan Rusu, Steve Cousins에 의해 공개된 Library를 나타내기도 합니다. 


여기서는 PCL, PCL-Python과 일부 Open3D를 활용합니다.





---
<a name="1">[1]</a> Radu Bogdan Rusu and Steve Cousins, "3D is here: Point Cloud Library (PCL)", IEEE International Conference on Robotics and Automation (ICRA), 2011 <br/>




