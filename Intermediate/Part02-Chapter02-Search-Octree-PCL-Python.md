
# PCL-Python 기반  Ocree 검색

> C++ 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Octree-PCL-Cpp.cpp)에서 다운로드 가능합니다. 원본 코드는 [[이곳]](https://github.com/strawlab/python-pcl/blob/master/examples/official/octree/octree_search.py)을 참고 하였습니다. 샘플파일은 [[cloud_cluster_0.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)을 사용하였습니다. Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-Search-Octree-PCL-Python.ipynb)에서 확인 가능 합니다. 



```python
!python --version 
!pip freeze | grep pcl 
```

    Python 2.7.15rc1
    python-pcl==0.3



```python
import pcl
import numpy as np
import random
```


```python
cloud = pcl.load("cloud_cluster_0.pcd")
```


```python
resolution = 0.2
octree = cloud.make_octreeSearch(resolution)
octree.add_points_from_input_cloud()
```

## SeartchPont 설정 
- 3000번째 포인트


```python
searchPoint = pcl.PointCloud()
searchPoints = np.zeros((1, 3), dtype=np.float32)
searchPoints[0][0] = cloud[3000][0]
searchPoints[0][1] = cloud[3000][1]
searchPoints[0][2] = cloud[3000][2]
#searchPoints = (cloud[3000][0], cloud[3000][1], cloud[3000][2])

searchPoint.from_array(searchPoints)
```

## Neighbors within voxel search


```python
ind = octree.VoxelSearch(searchPoint)
```

    VoxelSearch at (0.0346005521715 -1.46636068821 0.975462853909)



```python
print('Neighbors within voxel search at (' + str(searchPoint[0][0]) + ' ' + str(
    searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ')')

for i in range(0, 5):#range(0, ind.size):
    print('index = ' + str(ind[i]))
    print('(' + str(cloud[ind[i]][0]) + ' ' +
          str(cloud[ind[i]][1]) + ' ' + str(cloud[ind[i]][2]))
```

    Neighbors within voxel search at (0.0346005521715 -1.46636068821 0.975462853909)
    index = 412
    (-0.0524208694696 -1.53244829178 1.08694171906
    index = 461
    (-0.0523550510406 -1.5297921896 1.0849506855
    index = 508
    (-0.0461958646774 -1.51667225361 1.08676922321
    index = 509
    (-0.0491730645299 -1.52067470551 1.08531403542
    index = 510
    (-0.0522709041834 -1.52677381039 1.08309662342


## K nearest neighbor search


```python
K = 10
print('K nearest neighbor search at (' + str(searchPoint[0][0]) + ' ' + str(
        searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ') with K=' + str(K))
```

    K nearest neighbor search at (0.0346005521715 -1.46636068821 0.975462853909) with K=10



```python
[ind, sqdist] = octree.nearest_k_search_for_cloud(searchPoint, K)
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
[ind, sqdist] = octree.radius_search(searchPoints, radius, 10)
```

    Exception TypeError: 'only length-1 arrays can be converted to Python scalars' in 'pcl._pcl.to_point_t' ignored



```python
for i in range(0, ind.size):
    print('(' + str(cloud[ind[0][i]][0]) + ' ' + str(cloud[ind[0][i]][1]) + ' ' + str(
        cloud[ind[0][i]][2]) + ' (squared distance: ' + str(sqdist[0][i]) + ')')
```