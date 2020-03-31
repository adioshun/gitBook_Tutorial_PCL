# Smoothig-PCL-Python  \(70%\)

> C++ 코드는 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter06-Smoothig-PCL-Cpp.cpp)에서 다운로드 가능합니다. 원본 코드는 [\[이곳\]](https://github.com/strawlab/python-pcl/blob/master/examples/official/Surface/resampling.py)을 참고 하였습니다. 샘플파일은 [\[bunny.pcd\]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/bunny.pcd)을 사용하였습니다. Jupyter 버젼은 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter06-Smoothig-PCL-Cpp.ipynb)에서 확인 가능 합니다.

```python
!python --version 
!pip freeze | grep pcl
```

```text
Python 2.7.15rc1
python-pcl==0.3
```

```python
import numpy as np
import pcl
import random

cloud = pcl.load('bunny.pcd')
print('cloud(size) = ' + str(cloud.size))

# Create a KD-Tree
tree = cloud.make_kdtree()

# Output has the PointNormal type in order to store the normals calculated by MLS
mls = cloud.make_moving_least_squares()
mls.set_Compute_Normals (True)
mls.set_polynomial_fit (True)
mls.set_Search_Method (tree)
mls.set_search_radius (0.03) # Use all neighbors in a radius of 3cm.


# // Reconstruct
mls_points = mls.process ()

print('cloud(size) = ' + str(mls_points.size))

pcl.save_PointNormal(mls_points, 'bunny-mls.pcd')
```

```text
cloud(size) = 397
cloud(size) = 397
```

노이즈 제거후 Upsampling을 수행 하므로, 제거된 노이즈가 많을경우 포인트 수는 오히려 감소 할수 있습니다.

