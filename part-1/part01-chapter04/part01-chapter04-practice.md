# ROS 실습 \(90%\)

[![Alt text](https://img.youtube.com/vi/2DZRY71ZbJU/0.jpg)](https://www.youtube.com/watch?v=2DZRY71ZbJU)

실습에서는 **PCL-Python 기반 노이즈 제거**에서 정의한 `do_statistical_outlier_filtering()`를 사용하여 수신된 Raw데이터에서 노이즈를 제 하여 출력 해보도록 하겠습니다.

파라미터에 따라서 원거리 점군도 노이즈로 인식 하여 제거 될수 있습니다. 기본 구조는 이전챕터에서 살펴본 \[ROS 기반 I/O\]와 동일 합니다.

```python
#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k: number of neighboring points to analyze for any given point
    :param tresh: Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :", cloud, type(cloud))

    # 실행 코드 부분
    cloud = pcl_helper.XYZRGB_to_XYZ(cloud)

    mean_k = 10
    tresh = 0.001
    cloud = do_statistical_outlier_filtering(cloud,mean_k,tresh)

    color = pcl_helper.random_color_gen()
    cloud = pcl_helper.XYZ_to_XYZRGB(cloud,color)
    print("Output :", cloud, type(cloud))
    print("")


    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback)

    pub = rospy.Publisher("/velodyne_points_new", PointCloud2, queue_size=1)
    rospy.spin()
```

결과

```text
('Input :', <PointCloud of 19042 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)
('Output :', <PointCloud of 12803 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)

('Input :', <PointCloud of 19065 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)
('Output :', <PointCloud of 12840 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)

('Input :', <PointCloud of 19154 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)
('Output :', <PointCloud of 12915 points>, <type 'pcl._pcl.PointCloud_PointXYZRGB'>)
```

