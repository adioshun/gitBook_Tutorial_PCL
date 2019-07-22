#  Point cloud Library 설치 

앞에서 살펴 보았듯이 현재 PCL, PCL-python, Open3D, cilantro, pyPCD, Laspy, PCLpy 등 포인트 클라우드를 처리를 위한 많은 라이브러리 들이 있습니다. 개발 환경과 특성에 맞는 것을 골라 설치 하면 됩니다. 

본 튜토리얼에서는 PCL-C++, PCL-Python, Open3D-Python을 주로 사용하므로 이에 대한 설치 방법만 정리 하였습니다. 

또는, 위 라이브러리가 모두 설치되어 있는 Docker 이미지를 만들어 놓았습니다.  


## 1. PCL-C++ 설치 방법 

### 1.1 Package 설치 

```python
sudo apt-get update && sudo apt-get install -y software-properties-common git
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y && sudo apt-get update

sudo apt-get install -y libpcl-all #ubnutu 14
sudo apt-get install -y libpcl-dev #ubuntu 16 (libpcl-dev 1.7.2)
sudo apt-get install -y libpcl-dev #ubuntu 18
```

### 1.2 Source 설치 


```python
sudo apt-get update -qq && sudo apt-get install -y --no-install-recommends \
make cmake build-essential git \
libeigen3-dev \
libflann-dev \
libusb-1.0-0-dev \
libvtk6-qt-dev \
libpcap-dev \
libboost-all-dev \
libproj-dev \
&& sudo rm -rf /var/lib/apt/lists/*
#apt-get install git build-essential linux-libc-dev cmake cmake-gui libusb-1.0-0-dev libusb-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev libeigen3-dev libboost-all-dev libvtk5.10-qt4 libvtk5.10 libvtk5-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre

# ubuntu 16 (checked)
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar zvfx pcl-1.8.1.tar.gz

cd pcl-1.8.1
mkdir build && cd build
cmake .. # with enhanced compiler optimizations `cmake -DCMAKE_BUILD_TYPE=Release ..`
make -j2
sudo make -j2 install

# or 
$ git clone https://github.com/PointCloudLibrary/pcl.git
$ cd pcl && mkdir release && cd release
$ cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON -DCMAKE_INSTALL_PREFIX=/usr ..
$ make -j8
$ sudo make install

# or 
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake ..
make
sudo checkinstall -D make install #sudo make install대신 실행 추천 
#apt-get install checkinstall


$ sudo apt-get install ros-kinetic-pcl-conversions ros-kinect-pcl-ros
```


### 설치 확인 

```

cd ~ && mkdir pcl-test && cd pcl-test

wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/CMakeLists.txt 
wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/main.cpp
mkdir build && cd build
cmake .. && make && ./pcl-test
# Error
sudo ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9.1.0 /usr/lib/x86_64-linux-gnu/libproj.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
```


---

## 2. PCL-Python 설치 

설치 요구 사항

* PointCloudLibrary 1.6.x 1.7.x 1.8.x 1.9.x
* NumPy 1.9+
* Cython &gt;=0.25.2


### Package 설치

```
pip install python-pcl

#삭제 
pip uninstall python-pcl

#업그레이드 
pip install -U python-pcl

# 재설치 
$ pip uninstall python-pcl
$ pip install python-pcl --no-cache-dir
# You need to reinstall python-pcl when you want to upgrade PointCloudLibrary
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




