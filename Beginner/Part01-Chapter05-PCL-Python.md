
# PCL-Python 기반  바닥제거(RANSAC)

> C++ 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[tabletop_passthrough.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)을 사용하였습니다. Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Python.ipynb)에서 확인 가능 합니다. 원본 코드는 [[이곳]](https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/Plane_model_segmentation.py)을 참고 하였습니다.



```python
!python --version 
!pip freeze | grep pcl 
```

    Python 2.7.12
    /usr/local/lib/python2.7/dist-packages/pip/_vendor/requests/__init__.py:83: RequestsDependencyWarning: Old version of cryptography ([1, 2, 3]) may cause slowdown.
      warnings.warn(warning, RequestsDependencyWarning)
    [33mDEPRECATION: Python 2.7 will reach the end of its life on January 1st, 2020. Please upgrade your Python as Python 2.7 won't be maintained after that date. A future version of pip will drop support for Python 2.7.[0m
    python-pcl==0.3.0rc1



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
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    segmenter.set_max_iterations(100)
    segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, coefficients = segmenter.segment()
    
    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))
    
    print('Model inliers: ' + str(len(indices)))
    for i in range(0, 5):#range(0, len(indices)):
        print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
              str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliers, outliers
```


```python
indices, inliers, outliers= do_ransac_plane_normal_segmentation(cloud,0.05 )
```

    Model coefficients: 0.00566672021523 0.000429861887824 0.999983847141 -0.776584267616
    Model inliers: 48547
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
pcl.save(inliers_cloud, 'RANSAC_plane_true123.pcd.pcd') 
```