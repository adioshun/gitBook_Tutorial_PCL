# 실습 

실습에서는 ROS를 이용하여 직접 Velodyne Puck 16ch로 수집하고, Lidar 점군 데이터에 배운 내용을 적용해 보도록 하겠습니다. 

현재 라이다 장비가 연결되어 있지 않아도 상관 없습니다. ROS에서는 모든 센서 수집 정보 및 데이터를 `bag`이라는 파일에 압축 형태로 저장하여 쉽게 재생이 가능합니다. 

그리고 rviz라는 ros에서 제공하는 툴을 이용하여 시각화 할수 있습니다. 본 챕터에서는 ROS설치, bag파일 재생, 시각화에 대하여 살펴 보도록 하겠습니다. 

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

## 2. ROSbag 실행 






ROS파일에서 PCD파일 추출 하기

http://wiki.ros.org/pcl_ros

rosrun pcl_ros bag_to_pcd <input_file.bag> / <output_directory> rosrun pcl_ros bag_to_pcd lobby_velodyne.bag /velodyne_points ./lobby_pcd

