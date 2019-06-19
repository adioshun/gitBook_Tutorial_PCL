
# PCL-Python 기반  KdTree 검색

> C++ 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Kdtree-PCL-Cpp.cpp)에서 다운로드 가능합니다. 원본 코드는 [[이곳]](https://github.com/strawlab/python-pcl/blob/master/examples/official/kdtree/kdtree_search.py)을 참고 하였습니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Kdtree-PCL-Python.ipynb)에서 확인 가능 합니다. 



```python
!python --version 
!pip freeze | grep pcl 
```

    Python 2.7.15rc1
    python-pcl==0.3



```python
import numpy as np
import pcl
import random
import pcl_helper
```


```python
cloud = pcl.load("cloud_cluster_0.pcd")
```


```python
kdtree = cloud.make_kdtree_flann()
```

## SeartchPont 설정 
- 3000번째 포인트


```python
searchPoint = pcl.PointCloud()
searchPoints = np.zeros((1,3), dtype=np.float32)
searchPoints[0][0] = cloud[3000][0]
searchPoints[0][1] = cloud[3000][1]
searchPoints[0][2] = cloud[3000][2]

searchPoint.from_array(searchPoints)
```

## K nearest neighbor search


```python
K = 10
print('K nearest neighbor search at (' + str(searchPoint[0][0]) + ' ' + str(
        searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ') with K=' + str(K))
```

    K nearest neighbor search at (0.0346005521715 -1.46636068821 0.975462853909) with K=10



```python
[ind, sqdist] = kdtree.nearest_k_search_for_cloud(searchPoint, K)
```


```python
for i in range(0, ind.size):
    print('(' + str(cloud[ind[0][i]][0]) + ' ' + str(cloud[ind[0][i]][1]) + ' ' + str(
        cloud[ind[0][i]][2]) + ' (squared distance: ' + str(sqdist[0][i]) + ')')
```

    (0.0346005521715 -1.46636068821 0.975462853909 (squared distance: 0.0)
    (0.0317970663309 -1.46587443352 0.975684165955 (squared distance: 8.14496e-06)
    (0.0374080836773 -1.46704232693 0.975152671337 (squared distance: 8.44308e-06)
    (0.0345886982977 -1.46636962891 0.978524148464 (squared distance: 9.37174e-06)
    (0.0346124246716 -1.46635174751 0.972395777702 (squared distance: 9.40718e-06)
    (0.0373939499259 -1.46708440781 0.978200435638 (squared distance: 1.58212e-05)
    (0.0318062528968 -1.46585941315 0.972620844841 (squared distance: 1.61364e-05)
    (0.0317878909409 -1.46588945389 0.978741645813 (squared distance: 1.88836e-05)
    (0.0374225899577 -1.46703207493 0.972084701061 (squared distance: 1.98266e-05)
    (0.0289955306798 -1.4653942585 0.975902557373 (squared distance: 3.25436e-05)


## Neighbors within radius search


```python
radius = 0.02
print('Neighbors within radius search at (' + str(searchPoint[0][0]) + ' ' + str(
        searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ') with radius=' + str(radius))
```

    Neighbors within radius search at (0.0346005521715 -1.46636068821 0.975462853909) with radius=0.02



```python
[ind, sqdist] = kdtree.radius_search_for_cloud(searchPoint, radius)
```


```python
for i in range(0, ind.size):
    print('(' + str(cloud[ind[0][i]][0]) + ' ' + str(cloud[ind[0][i]][1]) + ' ' + str(
        cloud[ind[0][i]][2]) + ' (squared distance: ' + str(sqdist[0][i]) + ')')
```

---
[색상 변경 예제 추가]

```python 
float_rgb = pcl_helper.rgb_to_float([255,255,255])

points_list = []
for data in cloud_arr:
        points_list.append([data[0], data[1], data[2], float_rgb])

cloud = pcl.PointCloud_PointXYZRGB()
cloud.from_list(points_list)
```