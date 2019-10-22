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

# K3D를 이용한 동적 3D시각화 

Jupyter notebook extension for 3D visualization. [[홈페이지]](https://github.com/K3D-tools/K3D-jupyter)

설치 
```bash 
#PyPi
$ pip install k3d
 
#Conda
$ conda install -c conda-forge k3d

#Souce
$ pip install git+https://github.com/K3D-tools/K3D-jupyter

#Source 
$ git clone https://github.com/K3D-tools/K3D-jupyter.git
$ cd K3D-jupyter
$ pip install -e .

#주피터 적용 (필수)
$ jupyter nbextension install --py --sys-prefix k3d
$ jupyter nbextension enable --py --sys-prefix k3d
```

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

> Jupyter 지원, 포인트수가 많을경우 느려지거나 에러 발생 


##### 2D 시각화 
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

##### 3D 시각화 

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



## 0. [설치](http://docs.enthought.com/mayavi/mayavi/installation.html#installing-with-pip)

```python 
# python3 권장 
$ pip3 install mayavi
$ pip3 install PyQt5
```
> [Plot with Mayavi in Jupyter notebook on Docker for Mac](https://taku-y.github.io/mac-docker-jupyter-mayavi.html)



## 1. Test code
```python
from mayavi import mlab
mlab.init_notebook()
s = mlab.test_plot3d()
s
```

## 2. 실행 코드 

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


## 실행 코드 with BBox

```python 
import numpy as np
from mayavi import mlab

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3

def draw_lidar(pc, color=None, fig=None, bgcolor=(0,0,0), pts_scale=1, pts_mode='point', pts_color=None):
    ''' Draw lidar points
    Args:
        pc: numpy array (n,3) of XYZ
        color: numpy array (n) of intensity or whatever
        fig: mayavi figure handler, if None create new one otherwise will use it
    Returns:
        fig: created or used fig
    '''
    if fig is None: fig = mlab.figure(figure=None, bgcolor=bgcolor, fgcolor=None, engine=None, size=(1600, 1000))
    if color is None: color = pc[:,2]
    mlab.points3d(pc[:,0], pc[:,1], pc[:,2], color, color=pts_color, mode=pts_mode, colormap = 'gnuplot', scale_factor=pts_scale, figure=fig)
    
    #draw origin
    mlab.points3d(0, 0, 0, color=(1,1,1), mode='sphere', scale_factor=0.2)
    
    #draw axis
    axes=np.array([
        [2.,0.,0.,0.],
        [0.,2.,0.,0.],
        [0.,0.,2.,0.],
    ],dtype=np.float64)
    mlab.plot3d([0, axes[0,0]], [0, axes[0,1]], [0, axes[0,2]], color=(1,0,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[1,0]], [0, axes[1,1]], [0, axes[1,2]], color=(0,1,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[2,0]], [0, axes[2,1]], [0, axes[2,2]], color=(0,0,1), tube_radius=None, figure=fig)


    mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=62.0, figure=fig)
    return fig

def draw_gt_boxes3d(gt_boxes3d, fig, color=(1,1,1), line_width=1, draw_text=True, text_scale=(1,1,1), color_list=None):
    ''' Draw 3D bounding boxes
    Args:
        gt_boxes3d: numpy array (n,8,3) for XYZs of the box corners
        fig: mayavi figure handler
        color: RGB value tuple in range (0,1), box line color
        line_width: box line width
        draw_text: boolean, if true, write box indices beside boxes
        text_scale: three number tuple
        color_list: a list of RGB tuple, if not None, overwrite color.
    Returns:
        fig: updated fig
    ''' 
    num = len(gt_boxes3d)
    for n in range(num):
        b = gt_boxes3d[n]
        if color_list is not None:
            color = color_list[n] 
        if draw_text: mlab.text3d(b[4,0], b[4,1], b[4,2], '%d'%n, scale=text_scale, color=color, figure=fig)
        for k in range(0,4):
            #http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
            i,j=k,(k+1)%4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i,j=k+4,(k+1)%4 + 4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i,j=k,k+4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    #mlab.show(1)
    #mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=62.0, figure=fig)
    return fig


def get_3d_box(box_size, heading_angle, center):
    ''' Calculate 3D bounding box corners from its parameterization.

    Input:
        box_size: tuple of (l,w,h)
        heading_angle: rad scalar, clockwise from pos x axis
        center: tuple of (x,y,z)
    Output:
        corners_3d: numpy array of shape (8,3) for 3D box cornders
    '''
    def roty(t):
        c = np.cos(t)
        s = np.sin(t)
        return np.array([[c,  0,  s],
                         [0,  1,  0],
                         [-s, 0,  c]])

    R = roty(heading_angle)
    l,w,h = box_size
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2];
    y_corners = [h/2,h/2,h/2,h/2,-h/2,-h/2,-h/2,-h/2];
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2];
    corners_3d = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d[0,:] = corners_3d[0,:] + center[0];
    corners_3d[1,:] = corners_3d[1,:] + center[1];
    corners_3d[2,:] = corners_3d[2,:] + center[2];
    corners_3d = np.transpose(corners_3d)
    return corners_3d


box_size = [1.734255, 0.519638, 1.711274] #l,w,h
heading_angle = -1.549531
center = [4.509659, 1.092125, 46.151784]

box3d_from_label = get_3d_box(box_size, heading_angle, center)

input = np.load("./test.npy")
fig_lidar = draw_lidar(input)
fig = draw_gt_boxes3d([box3d_from_label], fig_lidar)

mlab.show()
```


