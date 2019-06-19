
# PCL-Python 기반  바닥제거(RANSAC)

> C++ 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[tabletop_passthrough.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)을 사용하였습니다. Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Python.ipynb)에서 확인 가능 합니다. 원본 코드는 [[이곳]](https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/Plane_model_segmentation.py)을 참고 하였습니다.



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
cloud = pcl.load("tabletop_passthrough.pcd")
print(cloud)
```

    <PointCloud of 72823 points>


## Create the segmentation object


```python
seg = cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE) #SACMODEL_PLANE
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.01)
seg.set_normal_distance_weight(0.01)
seg.set_max_iterations(1000)
indices, coefficients = seg.segment()
```


```python
print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))
```

    Model coefficients: 2.84193629341e-05 -0.000706289487425 0.999999761581 -0.776011049747



```python
print('Model inliers: ' + str(len(indices)))
for i in range(0, 5):#range(0, len(indices)):
    print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
          str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))
```

    Model inliers: 47249
    1027, x: 0.914367496967, y : -2.25295114517, z : 0.774702429771
    1028, x: 0.910148262978, y : -2.25295114517, z : 0.774702429771
    1029, x: 0.905929028988, y : -2.25295114517, z : 0.774702429771
    1030, x: 0.901709794998, y : -2.25295114517, z : 0.774702429771
    1031, x: 0.897490561008, y : -2.25295114517, z : 0.774702429771


## Indices to Point cloud


```python
inliers_cloud = pcl.PointCloud()
inliers = np.zeros((len(indices), 3), dtype=np.float32)

for i in range(len(indices)):
    inliers[i] = cloud[indices[i]]
inliers_cloud.from_array(inliers)
```


```python
pcl.save(inliers_cloud, 'RANSAC_plane_true.pcd.pcd') 
```