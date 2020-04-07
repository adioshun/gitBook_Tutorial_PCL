# Cling\_PCL

> 인터프리터처럼 사용할 수 있는 C++ 프로그램

* [CERN Root](https://root.cern.ch/) 프로젝트에서 공개 
  * A modular scientific software toolkit.
  * ROOT는 빅데이터를 다루기 위해서 만들어진 프레임워크 입니다
  * [ROOT가이드](https://opentutorials.org/module/2860)
* The Low Level Virtual Machine \(LLVM\)과 CLang의 라이브러리를 바탕

## 설치

* 소스 : [https://root.cern.ch/cling-build-instructions](https://root.cern.ch/cling-build-instructions)
* 바이너리 : [https://root.cern.ch/download/cling](https://root.cern.ch/download/cling)
* 쥬피터 추가 : `jupyter-kernelspec install --user cling-cpp17` \#cling-cpp11, cling-cpp14, cling-cpp17도 가능


---
# [jupyter-xeus xeus-cling](https://github.com/jupyter-xeus/xeus-cling)

![](https://github.com/jupyter-xeus/xeus-cling/raw/master/docs/source/xeus-cling.svg?sanitize=true)


```python 
$ conda create -n xeus-cling -c conda-forge cmake xeus=0.23.3 cling=0.6.0 clangdev=5.0 llvmdev=5 nlohmann_json cppzmq=4.3.0 xtl=0.6.9 pugixml cxxopts=2.1.1
$ conda activate xeus-cling
$ git clone https://github.com/jupyter-xeus/xeus-cling.git
$ cmake -D CMAKE_INSTALL_PREFIX=${CONDA_PREFIX} -D CMAKE_C_COMPILER=$CC -D CMAKE_CXX_COMPILER=$CXX -D CMAKE_INSTALL_LIBDIR=${CONDA_PREFIX}/lib -D DOWNLOAD_GTEST=ON
#cmake -D CMAKE_INSTALL_PREFIX=${CONDA_PREFIX} -D CMAKE_C_COMPILER=$CC -D CMAKE_CXX_COMPILER=$CXX -D CMAKE_INSTALL_LIBDIR=${CONDA_PREFIX}/lib -D DOWNLOAD_GTEST=ON -D PCL_INCLUDE_DIRS=/usr/include/pcl-1.8;/usr/include/eigen3;/usr/include/vtk-6.2 -D PCL_LIBRARY_DIRS=/usr/lib/x86_64-linux-gnu/ -D PCL_DIRS=/usr/lib/x86_64-linux-gnu -D pcl_DIRS=/usr/lib/x86_64-linux-gnu
#cmake -D CMAKE_INSTALL_PREFIX=${CONDA_PREFIX} -D CMAKE_C_COMPILER=$CC -D CMAKE_CXX_COMPILER=$CXX -D CMAKE_INSTALL_LIBDIR=${CONDA_PREFIX}/lib -D DOWNLOAD_GTEST=ON -D PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS} -D PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS} -D PCL_DIRS=${PCL_DIRS} -D pcl_DIRS=${pcl_DIRS}
$ make && make install
$ conda install notebook -c conda-forge

## test 
# ./cling '#include <stdio.h>' 'printf("Hello World!\n")'
# cling '#include <pcl/io/pcd_io.h>'
```

---


- https://docs.google.com/presentation/d/1gEZkGk8NIfAzY8YiVbbv5LgJ54Ob0QnSqJSH83b6w-U/edit#slide=id.p19
- https://drive.google.com/drive/folders/0B3z2yMwAC21NaHgzMlFaWGczOG8
- https://blog.jupyter.org/interactive-workflows-for-c-with-jupyter-fe9b54227d92
- https://isocpp.org/blog/2017/11/jupyter-cling-interactive-cpp
- https://code-ballads.net/generated-notebooks/cpp/repl_cling/notebooks/3_Advices_And_Gotchas.html
- [https://github.com/vgvassilev/cling](https://github.com/vgvassilev/cling)
- [https://drive.google.com/drive/folders/0B3z2yMwAC21NaHgzMlFaWGczOG8?usp=sharing](https://drive.google.com/drive/folders/0B3z2yMwAC21NaHgzMlFaWGczOG8?usp=sharing)

- \[Advices and Gotchas when using cling and jupyter\]\([https://code-ballads.net/generated-notebooks/cpp/repl\_cling/notebooks/3\_Advices\_And\_Gotchas.html](https://code-ballads.net/generated-notebooks/cpp/repl_cling/notebooks/3_Advices_And_Gotchas.html)\)

- How to use Point Cloud Library in cling interpreter?, [https://github.com/jolesinski/cling-pcl-tutorial](https://github.com/jolesinski/cling-pcl-tutorial)

