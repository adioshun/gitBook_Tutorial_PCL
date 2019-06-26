# PointNet++ 을 이용한 분류 기술 

[![Alt text](https://img.youtube.com/vi/QNVJEI_g4rw/0.jpg)](https://www.youtube.com/watch?v=QNVJEI_g4rw)




수행 단계 
- pre-processing
- custom TensorFlow op integration
- post-processing
- visualization



Open3D 활용 분야 

- Point cloud data loading, writing, and visualization. 
- Data preprocessing,(voxel-based downsampling)
- Point cloud interpolation(fast nearest neighbor search for label interpolation) 


활용 데이터넷 
- 학습 : Semantic3D 
- 추론 : Semantic3D + KITTI 


During both training and inference, PointNet++ is fed with fix-sized cropped point clouds within boxes, we set the box size to be 60m x 20m x Inf, with the Z-axis allowing all values. During inference with KITTI, we set the region of interest to be 30m in front and behind the car, 10m to the left and right of the car center to fit the box size. This allows the PointNet++ model to only predict one sample per frame.


차별점 : In PointNet++’s set abstraction layer, the original points are subsampled, and features of the subsampled points must be propagated to all of the original points by interpolation
- 기존 : This is achieved by 3-nearest neighbors search (called ThreeNN)--- 
- 변경 : Open3D uses FLANN to build KDTrees for fast retrieval of nearest neighbors, which can be used to accelerate the ThreeNN op.




















---

- [[설명] On point clouds Semantic Segmentation](http://www.open3d.org/index.php/2019/01/16/on-point-clouds-semantic-segmentation/?fbclid=IwAR1zYJfqJxwFarNKp7rBkLEjmhzHEuFQiTv2Yo9vmly2AFRZmii8oBnMM0k)

- [[코드] Semantic3D semantic segmentation with Open3D and PointNet++](https://github.com/IntelVCL/Open3D-PointNet2-Semantic3D), [[결과]](http://www.semantic3d.net/view_method_detail.php?method=PointNet2_Demo) : [참고코드]를 활용하여 Open3D로 작성된 코드  

- [[참고코드] PointNet2 for semantic segmentation of 3d points clouds](https://github.com/mathieuorhan/pointnet2_semantic), [[결과]](http://www.semantic3d.net/view_method_detail.php?method=pointnetpp_sem) : [원본코드]를 활용하여서 작성된 코드 

- [[원본코드] PointNet++](https://github.com/charlesq34/pointnet2)