# ROS 실습 \(90%\)

## ROS 실습 - 바닥제거 \(PCL-Python\)

[![Alt text](https://img.youtube.com/vi/6Z_drBX6Mn8/0.jpg)](https://www.youtube.com/watch?v=6Z_drBX6Mn8)

실습에서는 **PCL-Python 기반 바닥제거**에서 정의한 `do_ransac_plane_normal_segmentation()`를 사용하여 수신된 데이터에서 바닥을 제거 하여 출력 해보도록 하겠습니다.

> 실습에 사용되는 Lidar는 16채널로 점군이 sparse합니다. 따라서 바닥이 면으로 표현 되지 않고 타워형의 링으로 표현 됩니다.

필요에 따라서, 다운 샘플링, 관심영역 설정, 노이즈제거, 바닥제거를 모두 수행 하여도 됩니다. 여기서는 **관심영역 설정**후에 **바닥제거**를 수행하였습니다.

기본 구조는 이전챕터에서 살펴본 \[ROS 기반 I/O\]와 동일 합니다.

```python
#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliers, outliers

def callback(input_ros_msg):

    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :", cloud, type(cloud))

    # 실행 코드 부분 
    cloud = do_passthrough(cloud, 'x', 1.0, 20.0)
    cloud = do_passthrough(cloud, 'y', -7.0, 5.5)
    _, _, cloud = do_ransac_plane_normal_segmentation(cloud, 0.05)

    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback)

    pub = rospy.Publisher("/velodyne_points_new", PointCloud2, queue_size=1)

    rospy.spin()
```

## ROS 실습 - 바닥제거 \(PCL-Cpp\)

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
## http://wiki.ros.org/pcl/Tutorials



ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    #pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

    // Spin
    ros::spin ();
}
```

