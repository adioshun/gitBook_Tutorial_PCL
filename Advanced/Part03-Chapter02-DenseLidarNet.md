# DenseLidarNet

Generating Dense Lidar Data using cues from monocular image and sparse lidar data.

![](https://github.com/345ishaan/DenseLidarNet/raw/master/imgs/1.png)



## 설치 

```
python 2.7
pip install torch==0.4.1 -f https://download.pytorch.org/whl/cu80/stable
pip install https://download.pytorch.org/whl/torchvision-0.1.6-py2-none-any.whl
pip install 
```

## 학습 데이터 생성 (KITTI)

```
$ vi utils/datagen_v2.py

self.kitti_img_dir = '/media/adioshun/data/datasets/training/image_2/'
self.kitti_calib_dir = '/media/adioshun/data/datasets/training/calib/'
self.kitti_label_dir = '/media/adioshun/data/datasets/training/label_2/'
self.kitti_lidar_dir = '/media/adioshun/data/datasets/training/velodyne'

self.dump_dir = '/tmp/DenseLidarNet'
```


## 실행 
```
$ python code/scripts/init_state_dict.py -> init_state_dict.py
$ python train.py -tp1 ../data/lidar_pts -tp2 ../data/tf_lidar_pts -tp3 ../data/bbox_info-vp1 ../data/lidar_pts -vp2 ../data/tf_lidar_pts -vp3 ../data/bbox_info
```

---

- [[코드] DenseLidarNet](https://github.com/345ishaan/DenseLidarNet)