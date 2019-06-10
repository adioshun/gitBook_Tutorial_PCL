## Point cloud 란? 

Point cloud 은 Lidar 센나 RGB-D센서, Stereo camera 등으로 수집되는 점군(Point cloud)입니다. 여기서 점군이란 3차원 공간상에 퍼져 있는 여러 점(Point)의 집합(set=cloud)을 의미 합니다. 

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



## Point cloud Library 란? 

Point cloud Library란 Point cloud 처리를 위한 라이브러리입니다. 

현재 PCL, PCL-python, Open3D, pyPCD, Laspy, PCLpy 등의 라이브러리들이 사용되고 있습니다. 일반적으로 PCL하면 2011년 Radu Bogdan Rusu, Steve Cousins에 의해 공개된 [Library](http://pointclouds.org/)를 나타내기도 합니다. 

일반적으로 Point cloud 라이브러리들은 Point cloud의 파일 저장, 읽기, 잡음제거, 정합, 군집화, 분류, Feature계산 등의 기능을 제공합니다. 

추후 각 라이블러리들의 특징점에 대하여 정리 하도록 하겠습니다. `Appendix-Libraries`


여기서는 PCL[[^1]](#1), PCL-Python[[^2]](#2), Open3D[[^3]](#3)를 활용합니다.



## Point Cloud Data(PCD) Format 

Point Cloud은 `*. asc` , `*. cl3` , `*. clr` , `*. fls` , `*. fws` , `*. las` , `*. ptg` , `*. pts` , `*. ptx` , `*. txt` , `*. pcd` , `*. xyz` 등의 여러 데이터 포맷을 사용 가능합니다. 



---
<a name="1">[1]</a> Radu Bogdan Rusu and Steve Cousins, "3D is here: Point Cloud Library (PCL)", IEEE International Conference on Robotics and Automation (ICRA), 2011 

<a name="1">[1]</a> Andrew Straw가 개발한 PCL[1] 의 Python버젼 라이브러리입니다.[[홈페이지]](http://strawlab.github.io/python-pcl/)

<a name="1">[1]</a> Qian-Yi Zhou와 박재신 교수가 Intel Lab재직 시절 개발한 Point Cloud 라이브러리 입니다. C++와 Python을 지원합니다. [[홈페이지]](http://www.open3d.org/)










