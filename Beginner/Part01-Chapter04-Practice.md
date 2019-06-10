# 


## PCL-Python Statistical 

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


def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point
    :param tresh:   Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()



def callback(input_ros_msg):
    
    pcl_xyzrgb = pcl_helper.ros_to_pcl(input_ros_msg) 
    input_pcl_xyz = pcl_helper.XYZRGB_to_XYZ(pcl_xyzrgb)
    

    outlier_pcl_xyz = do_statistical_outlier_filtering(input_pcl_xyz,10, 0.001)
    outlier_pcl_xyzrgb = pcl_helper.XYZ_to_XYZRGB(outlier_pcl_xyz, [255,255,255])
    outlier_ros_msg = pcl_helper.pcl_to_ros(outlier_pcl_xyzrgb) 
    pub = rospy.Publisher("/velodyne_outlier", PointCloud2, queue_size=1)
    pub.publish(outlier_ros_msg)



if __name__ == "__main__":
    rospy.init_node('myopen3d_node', anonymous=True)
    rospy.Subscriber('/mmWaveDataHdl/RScan',
                     PointCloud2, callback)
    
    rospy.spin()
```