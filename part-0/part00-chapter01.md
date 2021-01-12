# \[별첨\] PCL & PCD란 \(100%\)

## Point cloud 란?

포인트 클라우드\(Point cloud\) Lidar 센서, RGB-D센서 등 으로 수집되는 데이터를 의미 합니다. 이러한 센서들은 아래 그림처럼 물체에 빛/신호를 보내서 돌아 오는 시간을 기록하여 각 빛/신호당 거리 정보를 계산 하고 하나의 포인트\(점\)을 생성 합니다.

![](http://www.irobotnews.com/news/photo/201705/10629_24038_64.png)

포인트 클라우드 3차원 공간상에 퍼져 있는 여러 포인트\(Point\)의 집합\(set cloud\)을 의미 합니다.

| ![](https://i.imgur.com/8kEIXdA.png) | ![](https://i.imgur.com/fQrysOa.png) |
| :--- | :--- |
| Lidar센서로 수집 Point cloud | RGB-D 센서로 수집된 Point Cloud |

점군은 2D 이미지와 다르게 깊이\(z\)정보를 가지고 있기 때문에 기본적으로 `N x 3` Numpy 배열로 표현 됩니다. 여기서 각 N 줄은 하나의 점과 맵핑이 되며 3\(x,y,z\) 정보를 가지고 있습니다.

Point Cloud는 기본적으로는 x,y,z 세개의 정보로만 표현 가능하지만 센서에서 제공되는 추가 정보가 있을경우 `N x 4`Numpy 배열로도 표현 가능 합니다. 예를 들어 Lidar 센서에서 수집한 정보의 경우는 `reflectance(반사도/반사시간?)`라는 정보가 추가될 수 있으며, RGB-D에는 Color정보가 추가 될수 있습니다.

> 참고로 일부 문서에서는 Lidar등에서 수집되는 데이터를 **unordered** Point cloud로 RGB-D 등에서 수집되는 데이터를 **Ordered** Point cloud로 표기 하는 경우도 있습니다. 이는 `N x 3` Numpy 배열에서 N의 순서\(order\)가 물체를 표현하는데 영향을 미치기 때문입니다.

## 이미지 데이터와 Point Cloud

| 이미지 데이터 | Point Cloud Data |
| :--- | :--- |
| ![](https://i.imgur.com/3jR1TTW.png) | ![](http://i.imgur.com/Bc13san.png) |

이미지 데이터에서 픽셀의 위치 정보는 항상 양수 입니다. 기준점은 왼쪽 위 부터이며 좌표값은 정수\(integer\)로 표현 합니다.

포인트 클라우드에서 점의 위치 정보는 양수 또는 음수 입니다. 기준점은 센서의 위치이며 좌표값은 실수\(Real number\)로 표현 합니다. 기준점의 앞/뒤는 x 좌표, 왼쪽/오른쪽은 y 좌표, 위/아래는 z좌표로 나타냅니다.

![](http://i.imgur.com/smzFU5N.png)

## Point cloud Library 란?

Point cloud Library란 Point cloud 처리를 위한 라이브러리입니다.

일반적으로 Point cloud 라이브러리들은 Point cloud의 파일 저장, 읽기, 잡음제거, 정합, 군집화, 분류, Feature계산 등의 기능을 제공합니다.

* pcl\_filters : 3D 점군 데이터에서 이상값과 노이즈 제거 등의 필터링
* pcl\_features : 점군 데이터로부터 3D 특징 추정 \(feature estimation\) 을 위한 수많은 자료 구조와 방법들
* pcl\_keypoints : Keypoint \(or interest point\) 을 검출하는 알고리즘 구현 \(BRISK, Harris Corner, NARF, SIFT, SUSAN 등\)
* pcl\_registration : 여러 데이터셋을 합쳐 큰 모델로 만드는 registration 작업 \(ICP 등\)
* pcl\_kdtree : 빠른 최근거리 이웃을 탐색하는 FLANN 라이브러리를 사용한 kdtree 자료 구조
* pcl\_octree : 점군 데이터로부터 계층 트리 구조를 구성하는 방법
* pcl\_segmentation : 점군으로부터 클러스터들로 구분하는 알고리즘들
* pcl\_sample\_consensus : 선, 평면, 실린더 등의 모델 계수 추정을 위한 RANSAC 등의 알고리즘들
* pcl\_surface : 3D 표면 복원 기법들 \(meshing, convex hulls, Moving Least Squares 등\)
* pcl\_range\_image : range image \(or depth map\) 을 나타내고 처리하는 방법
* pcl\_io : OpenNI 호환 depth camera 로부터 점군 데이터를 읽고 쓰는 방법
* pcl\_visualization : 3D 점군 데이터를 처리하는 알고리즘의 결과를 시각화

현재 PCL, PCL-python, Open3D, cilantro, pyPCD, Laspy, PCLpy 등의 라이브러리들이 사용되고 있습니다. 일반적으로 PCL하면 2011년 Radu Bogdan Rusu, Steve Cousins에 의해 공개된 [Library](http://pointclouds.org/)를 나타내기도 합니다.

추후 각 라이블러리들의 특징점에 대하여 정리 하도록 하겠습니다. `Appendix-Libraries`

여기서는 PCL, PCL-Python, Open3D, cilantro를 활용합니다.

## Point Cloud Data\(PCD\) Format

Point Cloud은 `*. asc` , `*. cl3` , `*. clr` , `*. fls` , `*. fws` , `*. las` , `*. ptg` , `*. pts` , `*. ptx` , `*. txt` , `*. pcd` , `*. xyz` 등의 여러 데이터 포맷으로 사용 가능합니다.

단순히 x,y,z정보만을 가진 `*.txt` 포맷을 이용하여도 되고, 헤더 정보와 x,y,z를 가진 `*. pcd`포맷을 이용하기도 합니다.

일반적으로 사용된는 `*. pcd`포맷은 Header와 Data 세션으로 나누어 집니다.

* Header : 전체 포인트 수, 데이터 타입, 크기 등의 정보
* Data : x,y,z 또는 x,y,z + 추가정보

```text
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
0.81915 0.32 0 4.2108e+06
0.97192 0.278 0 4.2108e+06
0.944 0.29474 0 4.2108e+06
...
```

주위깊게 보아야 할부분은 FIELDS가 `x,y,z,rgb`로 추가 정보로 색상 정보를 가지고 있습니다. 다른 `*.pcd`는 FIELDS가 `x,y,z,`로 rgb가 없이 사용될수 있습니다.

> 이 둘을 구분 하지 않고 `load_XYZRGB`로 파읽을 읽고, `save_XYZ`로 저장 한다면 색상 정보를 잃어버리게 되므로 조심 해야 합니다. 자세한 `*.pcd`파일 포맷에 대한 정보는 [\[여기\]](http://pointclouds.org/documentation/tutorials/pcd_file_format.php)에서 얻을수 있습니다.

[\[1\]](part00-chapter01.md) Radu Bogdan Rusu and Steve Cousins, "3D is here: Point Cloud Library \(PCL\)", IEEE International Conference on Robotics and Automation \(ICRA\), 2011

[\[2\]](part00-chapter01.md) Andrew Straw가 개발한 PCL\[1\] 의 Python버젼 라이브러리입니다.[\[홈페이지\]](http://strawlab.github.io/python-pcl/), [\[메뉴얼\]](https://python-pcl-fork.readthedocs.io/en/rc_patches4/index.html)

[\[3\]](part00-chapter01.md) Qian-Yi Zhou와 박재신 교수가 Intel Lab재직 시절 개발한 Point Cloud 라이브러리 입니다. C++와 Python을 지원합니다. [\[홈페이지\]](http://www.open3d.org/)

[\[4\]](part00-chapter01.md) A lean C++ library for working with point cloud data [\[홈페이지\]](https://github.com/kzampog/cilantro)

