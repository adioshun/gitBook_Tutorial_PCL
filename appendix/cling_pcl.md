# Cling\_PCL

## 1. 설치

```python 
# 사전 설치 
$ sudo apt install zeromq #ubuntu 16
$ sudo apt instal libzmq3-dev
$ wget https://gist.githubusercontent.com/katopz/8b766a5cb0ca96c816658e9407e83d00/raw/bc93fda1fe2fe5c6f45648ba131596134d92f7dc/setup-zeromq.sh

# 바이너리 다운로드 및 설치 
$ tar xvf cling_2020-03-11_ubuntu18.tar.bz2 # https://root.cern.ch/download/cling/
$ cd cling_2020-03-11_ubuntu18/share/cling/Jupyter/kernel/
$ pip3 install -e .

# 쥬피터 등록 
$ jupyter-kernelspec install --user cling-cpp17
# 필요시 PATH등록 
```

>  [소스 설치](https://root.cern.ch/cling-build-instructions) [[스크립트]](https://github.com/Axel-Naumann/cling-all-in-one)


## 2. 테스트 

### 2.1 print 

![](https://i.imgur.com/YKWnw9Q.png)

```cpp
#pragma cling add_library_path("/usr/lib/x86_64-linux-gnu")
#pragma cling load("libboost_system.so")
#pragma cling load("libpcl_io.so")
#pragma cling add_include_path("/usr/include/pcl-1.8")
#pragma cling add_include_path("/usr/include/eigen3")
#pragma cling add_include_path("/usr/include/vtk-6.3")

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);
std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";

```

### 2.2 [시각화](https://github.com/jolesinski/cling-pcl-tutorial)

![](https://i.imgur.com/kdKsB34.png)

```cpp
#pragma cling add_library_path("/usr/lib/x86_64-linux-gnu")
#pragma cling load("libpcl_visualization.so") // .L /usr/local/lib/libpcl_visualization.so

// Include dirs
#pragma cling add_include_path("/usr/include/pcl-1.8") // .I /usr/local/include/pcl-1.8
#pragma cling add_include_path("/usr/include/eigen3") // .I /usr/local/include/eigen3
#pragma cling add_include_path("/usr/include/vtk-6.3") // .I /usr/include/vtk-6.2

#include <pcl/visualization/pcl_visualizer.h>
auto viewer = pcl::visualization::PCLVisualizer("test");

viewer.setBackgroundColor(0,0,0);
viewer.addCoordinateSystem(0.5);
viewer.initCameraParameters();
viewer.addSphere(pcl::PointXYZ{1,1,1}, 0.2, 0.3, 0.3, 0.0, "sphere");
viewer.spin();


```


## 3. [도움말 ](https://xeus-cling.readthedocs.io/en/latest/magics.html)

```cpp
.? // help
.I <path> //Adds an include path;
.include //Show include path
.L <library | filename.cxx> // loads library or filename.cxx
.files //Show all loaded files
.q // quit ROOT
.L  file.C // Load  file.C
.x  file.C //Load  file.C  and run its function  file()
.U  file.C //Unload  file.C
.class C // Print what cling knows about class C
.O0 // (dot oh-zero) // Disable cling's optimization (for debugging)
.! cmd // Run shell command cmd
.@ //Cancels the multiline input
.printAST // (DEBUG ONLY) Turns on the printing of the compiler's abstract syntax tree (AST);
```

> [Advices and Gotchas when using cling and jupyter](https://code-ballads.net/generated-notebooks/cpp/repl_cling/notebooks/3_Advices_And_Gotchas.html)
