# DenseLidarNet

Generating Dense Lidar Data using cues from monocular image and sparse lidar data.

![](https://github.com/345ishaan/DenseLidarNet/raw/master/imgs/1.png)



## 설치 

```
python 2.7
pip install torch==0.4.1 -f https://download.pytorch.org/whl/cu80/stable
pip install https://download.pytorch.org/whl/torchvision-0.1.6-py2-none-any.whl
```

## 실행 
```
$ python code/scripts/init_state_dict.py -> init_state_dict.py
$ python train.py -tp1 ../data/lidar_pts -tp2 ../data/tf_lidar_pts -tp3 ../data/bbox_info-vp1 ../data/lidar_pts -vp2 ../data/tf_lidar_pts -vp3 ../data/bbox_info
```

---

- [[코드] DenseLidarNet](https://github.com/345ishaan/DenseLidarNet)