#  Point cloud Library 설치 

앞에서 살펴 보았듯이 현재 PCL, PCL-python, Open3D, cilantro, pyPCD, Laspy, PCLpy 등 포인트 클라우드를 처리를 위한 많은 라이브러리 들이 있습니다. 개발 환경과 특성에 맞는 것을 골라 설치 하면 됩니다. 

본 튜토리얼에서는 PCL-C++, PCL-Python, Open3D-Python을 주로 사용하므로 이에 대한 설치 방법만 정리 하였습니다. 

또는, 위 라이브러리가 모두 설치되어 있는 Docker 이미지를 만들어 놓았습니다.  

> ROS를 설치할경우 PCL 1.8.1(libpcl-dev)이 같이 설치 됩니다. (/usr/lib/x86_64-linux-gnu/)


## 1. PCL-C++ 설치 방법 

### 1.1 Package 설치 

```python
sudo apt-get update && sudo apt-get install -y software-properties-common git
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y && sudo apt-get update

sudo apt-get install -y libpcl-all #ubnutu 14
sudo apt-get install -y libpcl-dev #ubuntu 16 (libpcl-dev 1.7.2)
sudo apt-get install -y libpcl-dev #ubuntu 18 (PCL 1.8)
```








### 1.2 Source 설치 

#### A. 사전 패키지 설치 

```python
# 필수 설치 
$ sudo apt-get update -qq

$ sudo apt-get install -y --no-install-recommends make cmake cmake-gui build-essential git libeigen3-dev libflann-dev libusb-1.0-0-dev libboost-all-dev && 

# 추가 설치 
sudo apt-get update -qq && sudo apt-get install -y --no-install-recommends libflann1.8 libusb-dev libvtk6-qt-dev libpcap-dev libproj-dev linux-libc-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libvtk5.10-qt4 libvtk5.10 libvtk5-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre
  
 """
 git build-essential linux-libc-dev cmake cmake-gui libusb-1.0-0-dev libusb-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev libeigen3-dev libboost-all-dev libvtk5.10-qt4 libvtk5.10 libvtk5-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre
""" 

$ sudo rm -rf /var/lib/apt/lists/*


```

#### B. tar파일 다운으로 설치 

```python 
# ubuntu 16 (checked)
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar zvfx pcl-1.8.1.tar.gz

cd pcl-1.8.1
mkdir build && cd build
cmake .. # with enhanced compiler optimizations `cmake -DCMAKE_BUILD_TYPE=Release ..`
make -j2
sudo make -j2 install
```

#### C. git 으로 소스 설치 

 ```python  
$ git clone https://github.com/PointCloudLibrary/pcl.git
$ cd pcl && mkdir release && cd release
$ cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON -DCMAKE_INSTALL_PREFIX=/usr ..
$ make -j8
$ 
sudo make install
```

#### D. 사후 패키지 설치 

```python 
$ sudo apt-get install ros-kinetic-pcl-conversions ros-kinect-pcl-ros
```

### 1.3 설치 확인 

$cd ~ && mkdir pcl-test && cd pcl-test
$vi CMakeLists.txt

```python 
ADD_EXECUTABLE (pcd_write_test pcd_write.cpp) # (<생성될 실행 파일명>   <생성시 사용할 소스코드> )
```

```python 
#최소 요구 버젼 
CMAKE_MINIMUM_REQUIRED (VERSION 2.8 FATAL_ERROR) 

# 패키지 이름 
PROJECT (Part00-Chapter02)          
MESSAGE ( STATUS ${CMAKE_PROJECT_NAME} ) #( [<Type>] <메시지> )  

# 변수 설정 
SET(COMPILE_FLAGS "-std=c++11")  #SET ( <변수명> <값> )

# 의존성 패키지
FIND_PACKAGE (PCL 1.2 REQUIRED)  # 프로그램 실행시 필요한 패키지
                                # 없을 경우 에러 발생 

# 헤더 디렉토리 지정 (-I)
INCLUDE_DIRECTORIES(
    ${PCL_INCLUDE_DIRS}
    )

# 라이브러리 디렉토리 지정 (-L)   
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})


# 전처리기 매크로 추가 (-D)
ADD_DEFINITIONS(${COMPILE_FLAGS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})


# 생성할 실행 파일 옵션 
ADD_EXECUTABLE (pcd_write_test pcd_write.cpp) # (<생성될 실행 파일명>   <생성시 사용할 소스코드> )

# Target 링크 옵션 및 라이브러리 지정 (-l)
TARGET_LINK_LIBRARIES(pcd_write_test ${PCL_LIBRARIES})    #( <Target_이름> <라이브러리> <라이브러리> ... )
                                                    # 실행 파일생성하기에 앞서 링크 해야 하는 라이브러리와 실행 파일 링크 
```




$vi main.cpp
```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
```


$mkdir build && cd build
$cmake .. 
$make 
$./pcd_write_test

```
Saved 5 data points to test_pcd.pcd.
  0.352222 -0.151883 -0.106395
  -0.397406 -0.473106 0.292602
  -0.731898 0.667105 0.441304
  -0.734766 0.854581 -0.0361733
  -0.4607 -0.277468 -0.916762

```

> [Using PCL in your own project](http://www.pointclouds.org/documentation/tutorials/using_pcl_pcl_config.php)

---

## 2. PCL-Python 설치 

설치 요구 사항

* PointCloudLibrary 1.6.x 1.7.x 1.8.x 1.9.x
* NumPy 1.9+
* Cython &gt;=0.25.2


### Package 설치

```
sudo add-apt-repository ppa:sweptlaser/python3-pcl #Python3 Only??
sudo apt update
sudo apt install python3-pcl
```

### Source 설치

```python
pip3 install cython==0.25.2 && pip3 install numpy
git clone https://github.com/strawlab/python-pcl.git
cd python-pcl
python --version
sudo python3 setup.py build_ext -i
sudo python3 setup.py install

#pip3 install git+https://github.com/strawlab/python-pcl
```

---

## 3. Open3D

### Package 설치 

```python

sudo apt-get install -y libpng16-tools # libpng16-dev #libpng-dev 
sudo apt-get install -y libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libjpeg-dev python-dev python3-dev python-tk python3-tk

sudo apt-get install -y python3-pip 
pip3 install open3d-python
# or
pip3 install --user open3d-python
# or
python3 -m pip install --user open3d-python

#sudo apt-get install pybind11-dev # ~/Open3D/src/External/pybind11/build 
#sudo apt-get install xorg-dev 
```

```python 
#conda
conda install -c open3d-admin open3d
```

### Source 설치

```python
apt-get install cmake 
pip3 install numpy

cd ~
git clone https://github.com/IntelVCL/Open3D

# install the dependencies and use CMake to build the project
cd ~/Open3D
util/scripts/install-deps-ubuntu.sh

mkdir build
cd build
cmake ../src
make -j

# Install Python binding module 
cd util/scripts
./install.sh
```

---

## 4. cilantro


> 작성 중 


---

## 5. [PCL-To-All Docker](https://hub.docker.com/r/adioshun/pcl_to_all/) 이용 

Ubuntu 16, ROS, PCL-python, Open3D, Jupyter 등이 설치된 Docker를 다운 받아 바로 사용 할 수 있습니다. (2GB)


## 설치 및 실행

배포 페이지 : https://hub.docker.com/r/adioshun/pcl_to_all/

```
# 도커 이미지 받기 
$ docker push adioshun/pcl_to_all:20181122

#도커 실행 
$ docker run -it --net=host --volume /workspace:/workspace --name 'pcl_to_all' adioshun/pcl_to_all:20181122 /bin/bash
## --net=host : Host PC에서 ROS메시지를 받아 시각화 작업시 필요
## --volume : jupyter의 기본 작업 폴더로 Host PC와의 폴더 동기화를 위해 필요

# 컨테이너 실행 
$ docker start pcl_to_all

# 도커 접속 
$ docker exec -it pcl_to_all bash
```


Docker 내 쥬피터가 설치 되어 있습니다. 

```
# 쥬피터 실행 
$ jupyter notebook --allow-root

# 웹프라우져 접속 : http://localhost:8888 (접속 암호 : ubuntu)
```




