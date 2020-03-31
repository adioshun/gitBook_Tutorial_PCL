# PCL-Python-Helper \(10%\)

## pcl\_helper.py

[github](https://github.com/udacity/RoboND-Perception-Exercises#documentation-for-pcl_helperpy), [Code](https://github.com/udacity/RoboND-Perception-Exercises/tree/master/Exercise-2/sensor_stick/scripts)

`random_color_gen()` : Generates a random set of r,g,b values

* Return: a 3-tuple with r,g,b values \(range 0-255\)

`ros_to_pcl(sensor_msgs/PointCloud2)` : Converts sensor\_msgs/PointCloud2 to XYZRGB Point Cloud

* Return: pcl.PointCloud\_PointXYZRGB

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)`: Converts XYZRGB Point Cloud to sensor\_msgs/PointCloud2

* Return: sensor\_msgs/PointCloud2

`XYZRGB_to_XYZ(XYZRGB_cloud)`: Converts XYZRGB Point Cloud to XYZ Point CLoud

* Return: pcl.PointCloud

`XYZ_to_XYZRGB(XYZ_cloud, color)`:Takes a 3-tuple as color and adds it to XYZ Point Cloud

* Return: pcl.PointCloud\_PointXYZRGB

`rgb_to_float(color)`:Converts 3-tuple color to a single float32

* Return: rgb packed as a single float32

`get_color_list(cluster_count)` : Creates a list of 3-tuple \(rgb\) with length of the list = cluster\_count

* Return: get\_color\_list.color\_list

[/get\_PCD.py](https://github.com/CPFL/Autoware/blob/master/ros/src/util/packages/data_preprocessor/scripts/get_PCD.py)

[point\_cloud2.py](https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/src/point_cloud2.py)

코드 다운로드 : `wget https://gist.githubusercontent.com/adioshun/f35919c895631314394aa1762c24334c/raw/eb3b6493b964007f3103314e3208a48395f0f973/pcl_helper.py`

```python
from pcl_helper import *

color = random_color_gen()
xyzrgb = XYZ_to_XYZRGB(pc, color)
```

## Documentation for `pcl_helper.py`

`pcl_helper.py` contains useful functions for working with point cloud data with ROS and PCL. The file itself is located in `Exercise-2/sensor_stick/scripts/`. While the helper functions are required for Exercise-2, they could also come in handy if you want to explore more deeply in Exercise-1. Here's a brief description of the contents:

### Functions:

`random_color_gen()`

```text
Generates a random set of r,g,b values
Return: a 3-tuple with r,g,b values (range 0-255)
```

`ros_to_pcl(sensor_msgs/PointCloud2)`

```text
Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)`

```text
Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
Return: sensor_msgs/PointCloud2
```

`XYZRGB_to_XYZ(XYZRGB_cloud)`

```text
Converts XYZRGB Point Cloud to XYZ Point CLoud
Return: pcl.PointCloud
```

`XYZ_to_XYZRGB(XYZ_cloud, color)`

```text
Takes a 3-tuple as color and adds it to XYZ Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`rgb_to_float(color)`

```text
Converts 3-tuple color to a single float32
Return: rgb packed as a single float32
```

`get_color_list(cluster_count)`

```text
Creates a list of 3-tuple (rgb) with length of the list = cluster_count
Return: get_color_list.color_list
```

