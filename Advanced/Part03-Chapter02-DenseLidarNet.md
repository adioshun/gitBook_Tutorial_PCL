# DenseLidarNet

Generating Dense Lidar Data using cues from monocular image and sparse lidar data.

![](https://github.com/345ishaan/DenseLidarNet/raw/master/imgs/1.png)



## 설치 

```
python 2.7
pip install torch==0.4.1 -f https://download.pytorch.org/whl/cu80/stable
pip install https://download.pytorch.org/whl/torchvision-0.1.6-py2-none-any.whl
pip install tqdm h5py ipdb
```

## 학습 데이터 생성 (KITTI)

```
$ vi utils/datagen_v2.py

self.kitti_img_dir = '/media/adioshun/data/datasets/training/image_2/'
self.kitti_calib_dir = '/media/adioshun/data/datasets/training/calib/'
self.kitti_label_dir = '/media/adioshun/data/datasets/training/label_2/'
self.kitti_lidar_dir = '/media/adioshun/data/datasets/training/velodyne'

self.dump_dir = '../../data/'
```


## 실행 
```
$ python code/scripts/init_state_dict.py -> init_state_dict.py
$ python train.py -tp1 /tmp/DenseLidarNet/lidar_pts -tp2 /tmp/DenseLidarNet/tf_lidar_pts -tp3 /tmp/DenseLidarNet/bbox_info -vp1 /tmp/DenseLidarNet/lidar_pts -vp2 /tmp/DenseLidarNet/tf_lidar_pts -vp3 /tmp/DenseLidarNet/bbox_info

$ python train.py -e

```



## 에러 처리 







---

- [[코드] DenseLidarNet](https://github.com/345ishaan/DenseLidarNet)

- `train.py` 에러 수정 코드 [`Part03-Chapter02-DenseLidarNet_train.py`](./Part03-Chapter02-DenseLidarNet_train.py)







---


main.py의 `#transforms.Lambda(lambda x: logPolar_transform(x)),`를 주석 처리시 문제점 ?