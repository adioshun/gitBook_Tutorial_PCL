# ROS 기반 I/O

실행을 위해서는 ROS, PCL-Cpp, PCL-Python이 모두 설치 되어 있어야 합니다. [PCL-To-All Docker](https://hub.docker.com/r/adioshun/pcl_to_all/) 사용 하면 바로 실습이 가능합니다. 

ROS에서는 메시지를 토픽(topic)이라고 지칭 합니다. 아래 예제에서는 라이다 센서에서 입력되는 값을 'velodyne_points'이라는 토픽으로 입력 받고, 이를 그대로 다시 `/velodyne_points_new`이라는 토픽으로 출력하는 예제를 작성해 보겠습니다. 

```python 
#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper


def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)

    # 실행 코드 부분 
    print(cloud)

    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback)

    pub = rospy.Publisher("/velodyne_points_new", PointCloud2, queue_size=1)
    
    rospy.spin()
```

실행 
```
$ python Part00-Chapter03.py
```

시각화 
```
$ rviz -d lidar_new_topic.rviz
```

![](https://i.imgur.com/XWfezjK.png)


> 도커를 활용 하는 경우 `roscore`, `rviz`는 **Host PC**에서 실행하고, `python Part00-Chapter03.py`는 **Docker**에서 실행 하면 됩니다. `docker run -it --net=host`옵션으로 Host PC-Docker간 통신이 가능합니다. 