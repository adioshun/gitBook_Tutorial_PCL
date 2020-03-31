# PCL-Python  \(70%\)

> C++ 코드는 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [\[tabletop\_passthrough.pcd\]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)을 사용하였습니다. Jupyter 버젼은 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter05-PCL-Python.ipynb)에서 확인 가능 합니다. 원본 코드는 [\[이곳\]](https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/Plane_model_segmentation.py)을 참고 하였습니다.

```python
!python --version 
!pip freeze | grep pcl
```

```python
import pcl
import numpy as np
import random
```

```python
cloud = pcl.load("tabletop_passthrough.pcd")
print(cloud)
```

## do\_ransac\_plane\_segmentation

```python
def do_ransac_plane_segmentation(pcl_data,pcl_sac_model_plane,pcl_sac_ransac,max_distance):
    '''
    Create the segmentation object
    :param pcl_data: point could data subscriber
    :param pcl_sac_model_plane: use to determine plane models
    :param pcl_sac_ransac: RANdom SAmple Consensus
    :param max_distance: Max distance for apoint to be considered fitting the model
    :return: segmentation object
    '''
    seg = pcl_data.make_segmenter()
    seg.set_model_type(pcl_sac_model_plane)
    seg.set_method_type(pcl_sac_ransac)
    seg.set_distance_threshold(max_distance)
    return seg


def  extract_inlier_outlier(pcl_data,ransac_segmentation):
    '''
    :param pcl_data:
    :param ransac_segmentation:
    :return: cloud table and cloud object
    '''
    inliers, coefficients = ransac_segmentation.segment()
    inlier_object = pcl_data.extract(inliers, negative=False)
    outlier_object = pcl_data.extract(inliers, negative=True)
    return inlier_object,outlier_object
```

```python
# RANSAC Plane Segmentation
ransac_segmentation = do_ransac_plane_segmentation(cloud,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.01)

# Extract inliers and outliers
inlier, outliner = extract_inlier_outlier(cloud,ransac_segmentation )
```

## do\_ransac\_plane\_normal\_segmentation

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

```text
Model coefficients: 0.00566672021523 0.000429861887824 0.999983847141 -0.776584267616
Model inliers: 48547
1027, x: 0.914367496967, y : -2.25295114517, z : 0.774702429771
1028, x: 0.910148262978, y : -2.25295114517, z : 0.774702429771
1029, x: 0.905929028988, y : -2.25295114517, z : 0.774702429771
1030, x: 0.901709794998, y : -2.25295114517, z : 0.774702429771
1031, x: 0.897490561008, y : -2.25295114517, z : 0.774702429771
```

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

SACMODEL

```text
   48     SACMODEL_PLANE,
   49     SACMODEL_LINE,
   50     SACMODEL_CIRCLE2D,
   51     SACMODEL_CIRCLE3D,
   52     SACMODEL_SPHERE,
   53     SACMODEL_CYLINDER,
   54     SACMODEL_CONE,
   55     SACMODEL_TORUS,
   56     SACMODEL_PARALLEL_LINE,
   57     SACMODEL_PERPENDICULAR_PLANE,
   58     SACMODEL_PARALLEL_LINES,
   59     SACMODEL_NORMAL_PLANE,
   60     SACMODEL_NORMAL_SPHERE,
   61     SACMODEL_REGISTRATION,
   62     SACMODEL_REGISTRATION_2D,
   63     SACMODEL_PARALLEL_PLANE,
   64     SACMODEL_NORMAL_PARALLEL_PLANE,
   65     SACMODEL_STICK
```

