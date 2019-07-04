# ROS 기반 I/O

실행을 위해서는 ROS, PCL-Cpp, PCL-Python이 모두 설치 되어 있어야 합니다. [[Docker]](https://hub.docker.com/r/adioshun/pcl_to_all/) 사용 하면 바로 실습이 가능합니다. 

ROS에서는 메시지를 토픽(topic)이라고 지칭 합니다. 아래 예제에서는 라이다 센서에서 입력되는 값을 'velodyne_points'이라는 토픽으로 입력 받고, 이를 그대로 다시 `/velodyne_points_new`이라는 토픽으로 출력하는 예제를 작성해 보겠습니다. 

![](https://i.imgur.com/XWfezjK.png)


---

## 1. PCL-Python & ROS



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


> 도커를 활용 하는 경우 `roscore`, `rviz`는 **Host PC**에서 실행하고, `python Part00-Chapter03.py`는 **Docker**에서 실행 하면 됩니다. `docker run -it --net=host`옵션으로 Host PC-Docker간 통신이 가능합니다. 

---

## 2. PCL-Cpp & ROS

### 2.1 msg 송수신     


   
```cpp
$ cd ~/catkin_src/src/
$ catkin_create_pkg pcl_cpp_tutorial pcl pcl_ros roscpp sensor_msgs  #CMakeList.txt 자동 생성 
```

코드 작성 

```python 
// src/example.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>       // std::cout
#include <typeinfo>       // operator typeid

ros::Publisher pub;

typedef pcl::PointXYZRGBA              PointXYZRGBA;

void 
cloud_cb (const sensor_msgs::PointCloud2 msg)
{
  std::cout << "msg is: " << typeid(msg).name() << '\n';

  //ros_pcl2 to pcl2
  //http://www.pcl-users.org/ROS-PointCloud2-to-CloudXYZRGBA-td4039503.html
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(msg, pcl_pc);  
  std::cout << "cloud is: " << typeid(msg).name() << '\n';

  //pcl2 to pclxyzrgba
  pcl::PointCloud<PointXYZRGBA> input_cloud;
  pcl::fromPCLPointCloud2(pcl_pc, input_cloud);

  // Publish the data
  sensor_msgs::PointCloud2 output;
  output = msg;  
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
```



###### CMakeLists.txt
```
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
  )
  
  catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
                 pcl_conversions
                 pcl_ros
  )
  
  add_executable(roscpp_pcl_example src/roscpp_pcl_example.cpp)
  target_link_libraries(roscpp_pcl_example ${catkin_LIBRARIES})
  
  
```


```python
"""
target_link_libraries(<executableTargetName>, <lib1>, <lib2>, ... <libN>)


Example:

add_executable(foo src/foo.cpp)
add_library(moo src/moo.cpp)
target_link_libraries(foo moo)  -- This links foo against libmoo.so
"""
```


###### package.xml

```xml
  <build_depend>roscpp</build_depend>
  <build_depend>pcl_conversions</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>libpcl-all-dev</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>pcl_conversions</run_depend>
  <run_depend>pcl_ros</run_depend>
  <run_depend>libpcl-all</run_depend>



```


###### RUN 

```
catkin_make --directory ~/catkin_ws --pkg pcl_cpp_tutorial
source ~/devel/setup.sh
rosrun my_pcl_tutorial example input:=/narrow_stereo_textured/points2
```


#### [Tip] Conversion 



```cpp
// Converting pcl::PCLPointCloud2 to pcl::PointCloud and reverse
#include <pcl/conversions.h>
pcl::PCLPointCloud2 point_cloud2;
pcl::PointCloud<pcl::PointXYZ> point_cloud;
pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
pcl::toPCLPointCloud2(point_cloud, point_cloud2);

//Converting sensor_msgs::PCLPointCloud2 to sensor_msgs::PointCloud and reverse

#include <sensor_msgs/point_cloud_conversion.h>
sensor_msgs::PointCloud2 point_cloud2;
sensor_msgs::PointCloud point_cloud;

sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
sensor_msgs::convertPointCloud2ToPointCloud(point_cloud2, point_cloud);
```



```cpp
//Converting a PCL pointcloud to a ROS pcl message/ ROS pcl message to PCL point cloud
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//converting PCL pointcloud to a ROS PCL message:
sensor_msgs::PointCloud2 object_msg;
pcl::PointCloud::Ptr object_cloud;
object_cloud.reset(new pcl::PointCloud);
pcl::toROSMsg(*object_cloud.get(),object_msg );

//converting ROS PCL message to a PCL pointcloud:
pcl::PointCloud::Ptr received_cloud_ptr;
received_cloud_ptr.reset(new pcl::PointCloud);
sensor_msgs::PointCloud2ConstPtr pointcloud_msg;
pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());

```

- [Two ROS to PCL methods](http://www.programmersought.com/article/5428227702/)
- [How to use a PCL tutorial in ROS](http://wiki.ros.org/cn/pcl/Tutorials)






