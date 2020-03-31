# PCL-Python \(70%\)

> Jupyter 버젼은 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter01-PCL-Python.ipynb)에서 확인 가능 합니다.

## 1. 읽기

```python
import pcl

pc = pcl.load("./sample/lobby.pcd") # "pc.from_file" Deprecated
#cloud = pcl.load_XYZRGBA("tabletop.pcd")
print(pc)
```

## 2. 생성

```python
import pcl
import numpy as np


pc_array = np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32)
print(pc_array)

#방법 1
pc = pcl.PointCloud(pc_array)
print(pc)

#방법 2
pc = pcl.PointCloud()
pc.from_array(pc_array)
print(pc)


#방법 3 

searchPoint = pcl.PointCloud()
searchPoints = np.zeros((1, 3), dtype=np.float32) #np.zeros((1, 4) for RGBD
searchPoints[0][0] = 1024 * random.random() / (RAND_MAX + 1.0)
searchPoints[0][1] = 1024 * random.random() / (RAND_MAX + 1.0)
searchPoints[0][2] = 1024 * random.random() / (RAND_MAX + 1.0)


#방법 4 
p = pcl.PointCloud(10)  # "empty" point cloud
a = np.asarray(p)       # NumPy view on the cloud
a[:] = 0                # fill with zeros
print(p[3])             # prints (0.0, 0.0, 0.0)
a[:, 0] = 1             # set x coordinates to 1
print(p[3])             # prints (1.0, 0.0, 0.0)


#방법 5 for ROS
new_cloud = pcl.PointCloud()
new_cloud.from_array(new_data)
new_cloud = pcl_helper.XYZ_to_XYZRGB(new_cloud,[255,255,255])
```

## 3. 쓰기

```python
import pcl

# 방법 1
pcl.save(pc, 'pc2pcd.pcd') 
#pcl.save_XYZRGBA(pc, 'pc2pcd.pcd') #RGB-D센서에서 주로 사용, x,y,z좌표 이외 색상 정보 포함


# 방법 2
pc.to_file('pc2pcd.pcd')
```

## 4. 변환

추후 군집화, 분류, 전처리를 위해서 일반적으로 Numpy로 변환 하여 작업을 수행하므로 변환 과정에 대하여 살펴 보겠습니다.

```python
import pcl
import numpy as np

# PC to Numpy
pc_array = pc.to_array()

print("pc_array size : {}".format(pc_array.size))
print("pc Type : {}".format(type(pc)))
print("pc_array Type : {}".format(type(pc_array)))

# Numpy to PC 
pc_new = pcl.PointCloud()
pc_new.from_array(pc_array) # 2.생성 방법과 동일 #dtype=np.float32


# Indices to PC
cloud = pcl.load("tabletop_passthrough.pcd")

inliers_cloud = pcl.PointCloud()
inliers = np.zeros((len(indices), 3), dtype=np.float32)

for i in range(len(indices)):
    inliers[i] = cloud[indices[i]]
inliers_cloud.from_array(inliers)
```

## 5. 정보 출력

```python
import pcl
pc = pcl.load("./sample/lobby.pcd") 


print("포인트 수 : {}".format(pc)) 
print("포인트 수 : {}".format(pc.size)) 



# 포인트 값 
print ('Loaded ' + str(pc.width * pc.height) + ' data points from test_pcd.pcd with the following fields: ')

for i in range(0, 10):#pc.size):
    print ('x: ' + str(pc[i][0]) + ', y : ' + str(pc[i][1]) + ', z : ' + str(pc[i][2]))
```

