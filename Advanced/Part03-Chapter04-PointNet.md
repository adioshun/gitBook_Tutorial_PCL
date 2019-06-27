# PointNet++ 을 이용한 분류 기술 



[![Alt text](https://img.youtube.com/vi/QNVJEI_g4rw/0.jpg)](https://www.youtube.com/watch?v=QNVJEI_g4rw)

## 1. 개요 

https://adioshun.gitbooks.io/paper-3d-object-detection-and-tracking/content/2017-pointnet-deep-learning-on-point-sets-for-3d-classification-and-segmentation.html

## 2. 설치 

활용 데이터넷 
- 학습 : Semantic3D 
- 추론 : Semantic3D + KITTI 


During both training and inference, PointNet++ is fed with fix-sized cropped point clouds within boxes, we set the box size to be 60m x 20m x Inf, with the Z-axis allowing all values. During inference with KITTI, we set the region of interest to be 30m in front and behind the car, 10m to the left and right of the car center to fit the box size. This allows the PointNet++ model to only predict one sample per frame.


차별점 : In PointNet++’s set abstraction layer, the original points are subsampled, and features of the subsampled points must be propagated to all of the original points by interpolation
- 기존 : This is achieved by 3-nearest neighbors search (called ThreeNN)--- 
- 변경 : Open3D uses FLANN to build KDTrees for fast retrieval of nearest neighbors, which can be used to accelerate the ThreeNN op.

---

## 1. 수행 절차 

### 1.1 Post processing: accelerating label interpolation


|![](http://www.open3d.org/wordpress/wp-content/uploads/images/sparse.png)|![](http://www.open3d.org/wordpress/wp-content/uploads/images/dense.png)|
|-|-|
|Inference on sparse pointcloud (KITTI)|Inference results after interpolation|




The sparse labels need to be interpolated to generate labels for all input points. This interpolation can be achieved with nearest neighbor search using open3d.KDTreeFlann and majority voting, similar to what we did above in the ThreeNN op.

> 보간(interpolation)작업은 전체 소요 시간의 90%를 차지하고, 1FPS의 속도를 보인다. 해결을 위해 [custom TensorFlow C++ op InterploateLabel](https://github.com/intel-isl/Open3D-PointNet2-Semantic3D/blob/0de2ffe85e57f3dc8e06882731062b6a44721342/tf_ops/tf_interpolate.cpp#L118)를 적용하여 10+FPS속도 향상을 보였다. 


---

## 2. Install 

python3, tensorflow 1.2 (1.8권장??), open3d 0.6+

ubuntu 16.04, cuda 9.0 선호??

```
wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_9.0.176-1_amd64.deb
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
sudo dpkg -i cuda-repo-ubuntu1604_9.0.176-1_amd64.deb 
sudo apt-get update
sudo apt-get install cuda-9-0

```

### 2.1 Docker 

### 2.2 Code 

build TF ops. You’ll need CUDA and CMake 3.8+.

```
cd tf_ops
mkdir build
cd build
cmake -DCMAKE_C_COMPILER=/usr/bin/gcc ..
make

```


```
cd ./dataset
ln -s /media/adioshun/data/datasets/semantic3D/ ./semantic_raw



```








---

- [[설명] On point clouds Semantic Segmentation](http://www.open3d.org/index.php/2019/01/16/on-point-clouds-semantic-segmentation/?fbclid=IwAR1zYJfqJxwFarNKp7rBkLEjmhzHEuFQiTv2Yo9vmly2AFRZmii8oBnMM0k)

- [[코드] Semantic3D semantic segmentation with Open3D and PointNet++](https://github.com/IntelVCL/Open3D-PointNet2-Semantic3D), [[결과]](http://www.semantic3d.net/view_method_detail.php?method=PointNet2_Demo) : [참고코드]를 활용하여 Open3D로 작성된 코드  

- [[참고코드] PointNet2 for semantic segmentation of 3d points clouds](https://github.com/mathieuorhan/pointnet2_semantic), [[결과]](http://www.semantic3d.net/view_method_detail.php?method=pointnetpp_sem) : [원본코드]를 활용하여서 작성된 코드 

- [[원본코드] PointNet++](https://github.com/charlesq34/pointnet2)