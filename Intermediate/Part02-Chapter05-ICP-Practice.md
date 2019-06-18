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
       
    lidar_202 = pcl_helper.XYZRGB_to_XYZ(input_pcl)

    a202 = lidar_202.to_array()
    ones = np.ones((a202.shape[0],1))
    a202 = np.column_stack([a202, ones])


    key_array = np.array([[-0.986734, 0.070747, -0.146117, 13.072186],
                     [-0.051031, -0.989596, -0.134524, 0.532883],
                     [-0.154114, -0.125283, 0.980078, 1.061544],
                     [0.000000, 0.000000, 0.000000, 1.000000]])


    new_data =  np.ones((a202.shape[0],a202.shape[1]), dtype='f')

    for I in range(0,a202.shape[0]-1):
        new_data[I,] = np.dot(key_array, a202[I,])

    new_data = np.delete(new_data, (3), axis=1)
    new_data = np.delete(new_data, (new_data.shape[0]-1), axis=0)
    
    new_cloud = pcl.PointCloud()
    new_cloud.from_array(new_data)
    

    


    new_cloud = pcl_helper.XYZ_to_XYZRGB(new_cloud,[255,255,255])
    print("[{}]cloud type :{}".format(time.time(),type(new_cloud)))
    return new_cloud

def callback(input_ros_msg):
    
    pcl_xyzrgb = pcl_helper.ros_to_pcl(input_ros_msg) #ROS 메시지를 PCL로 변경    
    calibrated_pcl = icp(pcl_xyzrgb) # 탐지 영역(RoI) 설정 
    roi_ros_msg = pcl_helper.pcl_to_ros(calibrated_pcl) #PCL을 ROS 메시지로 변경 
    pub = rospy.Publisher("/velodyne_icp", PointCloud2, queue_size=1)
    pub.publish(roi_ros_msg)


if __name__ == "__main__":
    
    rospy.init_node('myopen3d_node', anonymous=True)
    rospy.Subscriber('/lidar_202/velodyne_points', PointCloud2, callback)    

    rospy.spin()
```