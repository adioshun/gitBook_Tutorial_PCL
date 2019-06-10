# Open3D-Python 기반 I/O

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter01-Open3D-Python.ipynb)에서 확인 가능 합니다. 


## 1. 읽기 

```python 
import open3d
import numpy as np


#PCD 파일 읽기
pc = open3d.read_point_cloud("./sample/lobby.pcd") 
print(pc)

#txt 파일 읽기
open3d.read_point_cloud("./sample/open3d_xyz.txt", format='xyz')

```

## 2. 생성 

```python 
import open3d
import numpy as np

pc_array = np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32)
print(pc_array)

pc = open3d.PointCloud()
pc.points = open3d.Vector3dVector(pc_array)
print(pc)
```


## 3. 쓰기 

지원 하는 확장자 :  pcd, ply, xyz, xyzrgb, xyzn, pts

```python 
import open3d

open3d.write_point_cloud("pc2pcd.pcd", pc)
```


## 4. 변환 

```python 
import open3d
import numpy as np

pc_array = np.asarray(pc.points)
print("pc Type : {}".format(type(pc)))
print("pc_array Type : {}".format(type(pc_array)))

#numpy를 pcd로 저장
pc_new = open3d.PointCloud()
pc_new.points = open3d.Vector3dVector(pc_array)
open3d.write_point_cloud("pc2pcd.pcd", pc_new)
```


## 5. 정보 출력  

```python 
import open3d
import numpy as np

print("포인트 수 : {}".format(pc.dimension))


#print ('Loaded ' + str(pc.width * pc.height) + ' data points from test_pcd.pcd with the following fields: ')

for i in range(0, 10):
    print ('x: ' + str(pc.points[i][0]) + ', y : ' + str(pc.points[i][1]) + ', z : ' + str(pc.points[i][2]))

```