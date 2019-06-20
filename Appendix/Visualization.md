# PCL-Cpp를 이용한 시각화 

- Cloud Viewer : 3D 동적 뷰어, 간단한 기능만을 가진 뷰어 [[참고]](https://adioshun.gitbooks.io/pcl/content/visualization/visualizing-point-clouds.html)
- Pcl Viewer : 3D 동적 뷰어, playing normals, drawing shapes, multiple viewport의 기능 가짐 [[참고]](https://adioshun.gitbooks.io/pcl/content/visualization/pclvisualizer.html)

##### code 시각화 

```cpp 
#include <pcl/visualization/cloud_viewer.h> 

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//...
pcl::visualization::CloudViewer viewer("cloud viewer");
viewer.showCloud(cloud);

while (!viewer.wasStopped ())
 {
 }

```



##### 프로그램 시각화 

```
$ sudo apt install pcl-tools 
$ pcl_viewer [파일명.pcd]
```

---

# PCL-Python을 이용한 시각화 

##### 색상 변경 예제 추가

```python 
float_rgb = pcl_helper.rgb_to_float([255,255,255])

points_list = []
for data in cloud_arr:
        points_list.append([data[0], data[1], data[2], float_rgb])

cloud = pcl.PointCloud_PointXYZRGB()
cloud.from_list(points_list)
```

---
# Open3D-Python을 이용한 시각화 

##### code 시각화 

##### Jupyter 시각화 

```python 
import numpy as np
import open3d as o3
from open3d import JVisualizer

pts_path = "table_scene_lms400.pcd"
fragment = o3.read_point_cloud(pts_path)
visualizer = JVisualizer()
visualizer.add_geometry(fragment)
visualizer.show()
```
---

# K3d를 이용한 동적 3D시각화 

```python 
pa = cloud.to_array()

plot = k3d.plot()
print(pa.shape[0])
points_number = pa.shape[0]
colors = np.random.randint(0, 0xFFFFFF, points_number)

points = k3d.points(pa, colors, point_size=0.01, shader='3d')
plot += points
plot.camera_auto_fit = False
plot.display()

```

---

# Matplot lib.를 이용한 시각화 

> Jupyter 지원 


##### 2D 
```python 
import matplotlib.pyplot as plt

def visualization2D_xyz(new_cloud_data):
    # set the size of pyplot charts
    plt.rcParams['figure.figsize'] = (14, 6)

    # split the rgb column into three columns: red, green and blue
    rgb_columns = np.asarray(random_color_gen())
    
    # normalize the rgb values (they should be between [0, 1])
    rgb_columns = (rgb_columns / 255.0).astype(np.float)

    # plot the points of the columns
    plt.scatter(new_cloud_data[:, [0]], -new_cloud_data[:, [1]], color=rgb_columns)

    # scale the axis equally 
    # (Note: append a semicolon to your last line to prevent jupyter notebook 
    # from outputting your last cell's content)
    plt.axis('scaled');
    print("(x) : {:2.1f}m".format(new_cloud_data[:,0:1].max() - new_cloud_data[:,0:1].min()))
    print("(y) : {:2.1f}m".format(new_cloud_data[:,1:2].max() - new_cloud_data[:,1:2].min()))
    print("(z) : {:2.1f}m".format(new_cloud_data[:,2:3].max() - new_cloud_data[:,2:3].min()))

%matplotlib inline
```

##### 3D 

```python 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



# Create a figure with a subplot with three axes

def visualization3D_xyz(new_cloud_data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # split the rgb column into three columns: red, green and blue
    rgb_columns = np.asarray(random_color_gen())
    
    # normalize the rgb values (they should be between [0, 1])
    rgb_columns = (rgb_columns / 255.0).astype(np.float)

    ax.scatter(new_cloud_data[:,0], new_cloud_data[:,1], new_cloud_data[:,2], color=rgb_columns);
    print("(x) : {:2.1f}m".format(new_cloud_data[:,0:1].max() - new_cloud_data[:,0:1].min()))
    print("(y) : {:2.1f}m".format(new_cloud_data[:,1:2].max() - new_cloud_data[:,1:2].min()))
    print("(z) : {:2.1f}m".format(new_cloud_data[:,2:3].max() - new_cloud_data[:,2:3].min()))

%matplotlib inline
```

사용 : `visualization3D_xyz(cloud.to_array())`


```python 

from random import randint
import struct
import ctypes

def random_color_gen():
    """ Generates a random color
    
        Args: None
        
        Returns: 
            list: 3 elements, R, G, and B
    """
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]
```

--- 

# Mayavi 이용 

> [Mayavi 홈페이지](http://docs.enthought.com/mayavi/mayavi/), [Plot with Mayavi in Jupyter notebook on Docker for Mac](https://taku-y.github.io/mac-docker-jupyter-mayavi.html), [공식 설치 가이드 /w Jupyter](http://docs.enthought.com/mayavi/mayavi/installation.html#installing-with-pip)




Test code
```python
from mayavi import mlab
mlab.init_notebook()
s = mlab.test_plot3d()
s
```

실행 코드 

```python
# ==============================================================================
#                                                                     VIZ_MAYAVI
# Input : kitti Raw Dataset 
# ==============================================================================
def viz_mayavi(points, vals="distance"):
    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    # r = lidar[:, 3]  # reflectance value of point
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor

    # Plot using mayavi -Much faster and smoother than matplotlib
    import mayavi.mlab

    if vals == "height":
        col = z
    else:
        col = d

    fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mayavi.mlab.points3d(x, y, z,
                         col,          # Values used for Color
                         mode="point",
                         colormap='spectral', # 'bone', 'copper', 'gnuplot'
                         # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                         figure=fig,
                         )
    mayavi.mlab.show()
```



---

# 



---

- Paraview : [ParaView/PCL Plugin](https://www.paraview.org/Wiki/ParaView/PCL_Plugin), [YouTube데모](https://www.youtube.com/watch?v=BZBQXcBvHW0) , `apt-get install paraview`

- [point cloud visualization with jupyter/pcl-python/and potree](https://www.youtube.com/watch?v=s2IvpYvB7Ew): YouTube데모 

- https://www.slicer.org/ : medical images


- http://www.sci.utah.edu/software/imagevis3d.html

- http://www.danielgm.net/cc/