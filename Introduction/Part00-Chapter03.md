# 실습 

실습에서는 ROS를 이용하여 직접 Velodyne Puck 16ch로 수집된 Lidar 점군 데이터에 배운 내용을 적용해 보도록 하겠습니다. 

현재 라이다 장비가 연결되어 있지 않아도 상관 없습니다. ROS에서는 모든 센서 수집 정보 및 데이터를 `bag`이라는 파일에 압축 형태로 저장하여 쉽게 재생이 가능합니다. 

그리고 rviz라는 ros에서 제공하는 툴을 이용하여 시각화 할수 있습니다. 본 챕터에서는 ROS설치, bag파일 재생, 시각화에 대하여 살펴 보도록 하겠습니다. 

마지막으로 PCL-Python을 ROS와 연동해 보겠습니다. 

---

## 1. ROS 설치 


ROS에는 Robotics Operating System의 약어입니다. OS라고는 하지만 Ubuntu상에서 동작하는 프로그램이라고 보시면 됩니다. 따라서 윈도우에서는 동작을 하지 않으며 ubuntu만 지원 합니다. 설치되는 Ubunut 버젼에 따라 지원하는 ROS도 서로 다릅니다. 현재 많이 사용되는 ubuntu 18에서는 Melodic이라는 ROS를 사용 합니다. 설치 방법은 인터넷상에 많이 공개 되어 있으며 ROS 공식 [[WIKI]](http://wiki.ros.org/ROS/Installation)를 참조 하셔도 됩니다. 여기서는 가장 간단한 스크립트 기반 설치 방법만 기술 하도록 하겠습니다. 


```
# Ubuntu 18용 Melodic 
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
$ chmod +x install_ros_melodic.sh
$ ./install_ros_melodic.sh

# Ubuntu 16용 Kinetic 
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
$ chmod +x install_ros_melodic.sh
$ ./install_ros_melodic.sh
```


설치가 완료된 후에는 아래 명령어도 정상 설치 여부를 확인 할 수 있습니다. 
```
$ roscore
```

아래와 같은 메시지가 출력 된다면 정상적으로 설치된 것입니다. 

```
SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3

NODES

auto-starting new master
process[master]: started with pid [20453]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 79bbdd7e-931d-11e9-8db3-107b44b179dc
process[rosout-1]: started with pid [20464]
started core service [/rosout]
```

ROS에서 제공한는 기능을 사용하기 위해서는 roscore가 항상 동작 하고 있어야 합니다. 따라서, 다음에 살펴볼 rosbag과 rviz를 실습시에는 새 터미널 창을 실행하고 거기서 명령어를 입력 하시기 바랍니다. 

> 창을 여러개 실행 시키는것이 싫으신 분은 터미널 창 분활을 지원하는 [Terminator] 터미널을 추천 합니다. `sudo apt-get install terminator`

---

## 2. ROSbag 실행 

rosbag은 ROS에서 제공하는 로깅 툴입니다. rosbag을 이용하여서 센서에서 들어는 모든 메시지들은 시간 순서에 맞게 저장 하고 재생 할 수 있습니다. 저장시 확장자는 `*.bag`입니다. 자세한 내용은 [[여기]](http://wiki.ros.org/rosbag)에서 확인 가능 합니다. 



실습을 위해서는 재생기능만 사용 할수 있으면 됩니다. roscore가 실행된 터미널 창외에 새로운 터미널을 실행 후 수행 하면 됩니다. 

명령어는 아래와 같습니다. 

```bash 
$ rosbag play -l lobby_lidar.bag
```
`-l` 옵션은 반복 재생으로 `--loop`를 사용 하셔도 됩니다. 


아래와 같은 메시지가 출력 된다면 정상적으로 실된 것입니다. 


```
[ INFO] [1561011516.035810913]: Opening lobby_lidar.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1561011373.074575   Duration: 1.915948 / 59.119541 
```

스페이스키를 누루면 일시 정지가 되고, `Ctrl + c`를 누루면 실행 취소가 됩니다. 


참고로 bag파일에서 pcd파일을 추출 하시려면 아래의 명령어를 이용 하면 됩니다. 
```
$ rosrun pcl_ros bag_to_pcd lobby_lidar.bag /velodyne_points ./lobby_pcd
# rosrun pcl_ros bag_to_pcd <input_file.bag> / <output_directory> 
```

* [ROS bags-TO-Image.ipynb](https://gist.github.com/anonymous/4857f8920c9fc901121a429ead32a7db)
* [ROS bags-TO-Point Clods.ipynb](https://gist.github.com/anonymous/e675ea14113252be321320be62248034)
* [ROS bags-TO-Avi.ipynb](https://gist.github.com/anonymous/fb1e98efe187b2a35b6d91fb5df9e83b)


해당 bag파일의 정보를 확인 하시려면 `rosbag info lobby_lidar.bag`를 실행 하시면 됩니다. 

```
path:        lobby_lidar.bag
version:     2.0
duration:    59.1s
start:       Jun 20 2019 15:16:11.16 (1561011371.16)
end:         Jun 20 2019 15:17:10.28 (1561011430.28)
size:        334.6 MB
messages:    587
compression: none [294/294 chunks]
types:       sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:      /velodyne_points   587 msgs    : sensor_msgs/PointCloud2
```

여기서 중요한것은 마지막 줄의 **topics**입니다. ROS에서는 모든 메시지를 **토픽(topic)**이라고 지칭 합니다. 


---


## 3. rviz를 이용한 시각화 

rviz는 ROS에서 제공하는 시각화 툴입니다. rviz를 이용하여서 센서에서 수집되는 값을 시각화 하거나, rosbag으로 재생하는 내용을 시각화 할 수 있습니다. 자세한 내용은 [[여기]](http://wiki.ros.org/rviz)에서 확인 가능 합니다. 

roscore, rosbag이 실행된 터미널 창외에 새로운 터미널을 실행 후 수행 하면 됩니다. 

명령어는 아래와 같습니다. 



```
rviz
rviz -d lidar.rviz
```
- '-d`옵션은 설정 파일을 이용하여 rviz를 실행 한것입니다. 


아래와 같은 창이 생성 된다면 정상적으로 실행된 것입니다. 

![](https://i.imgur.com/grI2aLP.png)


실행시 `rviz` 명령어로만 실행 하였다면 아무것도 보이지 않을것입니다. 어떤 메시지를 시각화 할지를 왼쪽 판넬에서 지정해 주어야 합니다. 
- Global Option - Fixed Frame : velodyne 
- Pointcloud2 - Topic : /velodyne_points 
 - Pointcloud2가 없다면 : 하단 Add - By display type - pointcloud2 - Ok 

