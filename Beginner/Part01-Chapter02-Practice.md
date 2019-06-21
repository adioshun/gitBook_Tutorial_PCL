# ROS 기반 Voxelization


실습에서는 **PCL-Python 기반 Voxelization**에서 정의한 `do_voxel_grid_downssampling()`를 사용하여 수신된 Raw데이터를 복셀화 하여 출력 해보도록 하겠습니다. 



```python 

#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
#import pcl_msg

import pcl_helper

def do_voxel_grid_downssampling(pcl_data,leaf_size):
    '''
    Create a VoxelGrid filter object for a input point cloud
    :param pcl_data: point cloud data subscriber
    :param leaf_size: voxel(or leaf) size
    :return: Voxel grid downsampling on point cloud
    :https://github.com/fouliex/RoboticPerception
    '''
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
    return  vox.filter()

def callback(input_ros_msg):

    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :", cloud)

    LEAF_SIZE = 0.1 
    cloud = do_voxel_grid_downssampling(cloud,LEAF_SIZE)
    print("Output :", cloud)
    print("")


    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)
    rospy.Subscriber('velodyne_points', PointCloud2, callback)

    pub = rospy.Publisher("/velodyne_points_new", PointCloud2, queue_size=1)
    


    rospy.spin()




```




결과값 
```
('Input :', <PointCloud of 18466 points>)
('Output :', <PointCloud of 10131 points>)

('Input :', <PointCloud of 18461 points>)
('Output :', <PointCloud of 10121 points>)
```


---
# [with ROS](https://github.com/rjw57/pcl_fuse/tree/master/src)

```cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

// This is to save on typing
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);
    
    // Convert point cloud to PCL native point cloud
    point_cloud_t::Ptr input_ptr(new point_cloud_t());
    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);
    
    // Set up VoxelGrid filter to bin into 10cm grid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_ptr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    
    // Create output point cloud
    point_cloud_t::Ptr output_ptr(new point_cloud_t());
    
    // Run filter
    sor.filter(*output_ptr);
    
    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);
    
    // Publish the data
    pub.publish(ros_output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_voxel");
    ros::NodeHandle nh;
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);
    
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/fused_points", 1);
    
    // Spin
    ros::spin ();
}

```

