




- voxelnet


### 데이터 준비 

```
└── DATA_DIR
       ├── training   <-- training data
       |   ├── image_2
       |   ├── label_2
       |   └── velodyne
       └── validation  <--- evaluation data
       |   ├── image_2
       |   ├── label_2
       |   └── velodyne
```

### 도커 pull & 실행 

```
$ docker pull adioshun/voxelnet
$ 
```


### 설정 수정(In the docker )

1. `config.py`

```
# for dataset dir
__C.DATA_DIR = '/voxelnet/data/dataset'
__C.CALIB_DIR = '/voxelnet/data/dataset/training/calib'


# for gpu allocation
__C.GPU_AVAILABLE = '0,1'
__C.GPU_USE_COUNT = len(__C.GPU_AVAILABLE.split(','))
__C.GPU_MEMORY_FRACTION = 1

```
1. `kitti_eval/launch_test.sh`




### train 

```
python3 train.py --alpha 1 --beta 10
```

- log 저장 위치  : `log/default` #Tensorboard 지원 
- validation results : `predictions/{epoch number}/data`
- model 저장 위치 : `save_model/default`
- 학습된 model 저장 위치 :`save_model/pre_trained_car`

>  Nvidia 1080 Ti GPUs로 약 3일이 소요 되므로 학습된 모델 사용을 권장 합니다. 

학습 완료 후 **Learning Curve** 확인 
```
python3 parse_log.py predictions
# predictions.jpg 생성 
```

### Evaluate

```
$ python3 test.py -n default #학습된 결과물 활용 
$ python3 test.py -n pre_trained_car #사전 학습된 결과물 활용 `save_model/pre_trained_car`
```

- 결과 저장 폴더 : `predictions/data`

### 결과 확인 

```
./kitti_eval/evaluate_object_3d_offline ./../data/dataset/validation/label_2/ ./prediction
```

---

- [an unofficial inplementation of VoxelNet in TensorFlow](https://github.com/qianguih/voxelnet)