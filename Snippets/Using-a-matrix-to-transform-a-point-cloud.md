# [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Snippets/Using-a-matrix-to-transform-a-point-cloud.ipynb)에서 확인 가능 합니다. 


## 1. using a Matrix4

```
  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
```

```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("room_scan1.pcd", *source_cloud);

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  /*
  0.998086 0.0580758 -0.0212953 -0.0897905
  -0.0579691 0.998303 0.00559049 0.043843
  0.0215839 -0.00434538 0.999758 -0.0233652
  0 0 0 1
  */

  // Define a translation of 2.5 meters on the x axis.
  transform_1 (0,0) = 0.998086;transform_1 (0,1) = 0.0580758;transform_1 (0,2) = -0.0212953;transform_1 (0,3) = -0.0897905;
  transform_1 (1,0) = -0.0579691;transform_1 (1,1) = 0.998303;transform_1 (1,2) = 0.00559049;transform_1 (1,3) = 0.043843;
  transform_1 (2,0) = 0.0215839;transform_1 (2,1) = -0.00434538;transform_1 (2,2) = 0.999758;transform_1 (2,3) = -0.0233652;
  transform_1 (3,0) = 0.0;transform_1 (3,1) = 0.0;transform_1 (3,2) = 0.0;transform_1 (3,3) = 1.0;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);  
  pcl::io::savePCDFile<pcl::PointXYZ>("room_scan1_transformed_cloud.pcd", *transformed_cloud); 

  // Visualization

  return 0;
}

```


---

## 2. Using a Affine3f

- This method is easier and less error prone


```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("room_scan1.pcd", *source_cloud);

  float theta = M_PI/4; // The angle of rotation in radians

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);  
  pcl::io::savePCDFile<pcl::PointXYZ>("room_scan1_transformed_cloud.pcd", *transformed_cloud); 

  // Visualization



  return 0;
}

```


---


## Tip. 바로 시각화 

white  = original point cloud
red  = transformed point cloud

```cpp 

  // Visualization

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

```

```
$ pcl_viewer room_scan1.pcd room_scan1_transformed_cloud.pcd
$ pcl_viewer room_scan1.pcd room_scan2.pcd 
```


|![](https://i.imgur.com/BLX32n2.png)|![](https://i.imgur.com/u6uushL.png)|
|-|-|
|||


# ROS 사용자용 

ROS에서는 자체 명령어로 좌표계 변환을 지원 하고 있습니다. 

```python
rosrun tf static_transform_publisher 0 0 0 0 0 0 velodyne velodyne_201 10
# rosrun tf static_transform_publisherx y z yaw pitch roll frame_id child_frame_id period_in_ms
# static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
```

여기서 yaw, pitch, roll는 각 z,y,x의 Rotation 정도 입니다. qx, qy, qz, qw는 quaternion표기값입니다. 

각 단위는 아래와 같습니다. 

|x,y,z|Meter||
|-|-|-|
|yaw, pitch, roll|Radians||
|period|ms.|100ms (10hz)추천|

- Rotation matrix Vs. Euler angle 변환 [코드](https://www.learnopencv.com/rotation-matrix-to-euler-angles/), [웹사이트](https://www.andre-gaschler.com/rotationconverter/) ,[시각화검증](http://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html)

- http://www.lfd.uci.edu/~gohlke/code/transformations.py.html 


---

## ROS 

```python 
#!/usr/bin/env python3
# coding: utf-8

import sys
sys.path.append("/workspace/include")

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import numpy as np
import pcl
import pcl_msg

import pcl_helper
import filter

import time

def icp(input_pcl):
    # 입력 포인트 #cloudB cloudA      
    lidar_201 = pcl_helper.XYZRGB_to_XYZ(input_pcl)

    a201= lidar_201.to_array()
    ones = np.ones((a201.shape[0],1))
    a201 = np.column_stack([a201, ones])

    key_array = np.array(
    [[-0.999471, 0.031399, -0.008493, 3.179482],
    [-0.031401, -0.999507, -0.000000, 14.970945],
    [-0.008489, 0.000267, 0.999964, -0.098983],
    [0.000000, 0.000000, 0.000000, 1.000000]]
    )



    new_data =  np.ones((a201.shape[0],a201.shape[1]), dtype='f')

    for I in range(0,a201.shape[0]-1):
        new_data[I,] = np.dot(key_array, a201[I,])

    new_data = np.delete(new_data, (3), axis=1)
    new_data = np.delete(new_data, (new_data.shape[0]-1), axis=0)
    
    new_cloud = pcl.PointCloud()
    new_cloud.from_array(new_data)
    

    


    new_cloud = pcl_helper.XYZ_to_XYZRGB(new_cloud,[255,255,255])
    #print("[{}]cloud type :{}".format(time.time(),type(new_cloud)))
    return new_cloud

def callback(input_ros_msg):
    
    pcl_xyzrgb = pcl_helper.ros_to_pcl(input_ros_msg) #ROS 메시지를 PCL로 변경    
    calibrated_pcl = icp(pcl_xyzrgb) # 탐지 영역(RoI) 설정 
    roi_ros_msg = pcl_helper.pcl_to_ros(calibrated_pcl) #PCL을 ROS 메시지로 변경 
    pub = rospy.Publisher("/velodyne_icp", PointCloud2, queue_size=1)
    pub.publish(roi_ros_msg)


if __name__ == "__main__":
    
    rospy.init_node('myopen3d_node', anonymous=True)
    rospy.Subscriber('/velodyne_bg_202', PointCloud2, callback)    
    """ 
    trans적용 대상(sorce) : 201
    target :202

    """

    rospy.spin()
```
