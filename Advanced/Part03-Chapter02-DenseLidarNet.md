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

train.py원본 코드를 

```python 
def train(self, epoch):
	
	train_loss = []
	self.net.train()
	
	for batch_idx,(voxel_features,voxel_mask,voxel_indices, chamfer_gt) in enumerate(self.train_dataloader):
		zipped_input = self.prepare_gpu_input(voxel_features, voxel_mask, voxel_indices)		
			
		hallucinations = self.customdataparallel(zipped_input)
		chamfer_gt = torch.FloatTensor(chamfer_gt)
		loss = self.criterion(hallucinations, Variable(chamfer_gt).cuda(self.gather_device))
		train_loss += [loss.data[0]/chamfer_gt.size(0)] #For pytorch <= 0.5
		#train_loss += [loss.data/chamfer_gt.size(0)]
		if batch_idx % self.args.print_freq == 0:
			progress_stats = '(train) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {net_loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.train_dataloader), net_loss=loss.data[0])
			print(progress_stats)
		self.optimizer.zero_grad()
		
		loss.backward()
		self.optimizer.step()
	return train_loss

	
```
아래와 같이 수정 

```python 
def train(self, epoch):
	
	train_loss = []
	self.net.train()
	
	for batch_idx,(voxel_features,voxel_mask,voxel_indices, chamfer_gt) in enumerate(self.train_dataloader):
		zipped_input = self.prepare_gpu_input(voxel_features, voxel_mask, voxel_indices)		
		if len(zipped_input) == 2:
			hallucinations = self.customdataparallel(zipped_input)
			chamfer_gt = torch.FloatTensor(chamfer_gt)
			loss = self.criterion(hallucinations, Variable(chamfer_gt).cuda(self.gather_device))
			train_loss += [loss.data[0]/chamfer_gt.size(0)] #For pytorch <= 0.5
			#train_loss += [loss.data/chamfer_gt.size(0)]
			if batch_idx % self.args.print_freq == 0:
				progress_stats = '(train) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {net_loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.train_dataloader), net_loss=loss.data[0])
				print(progress_stats)
			self.optimizer.zero_grad()
			
			loss.backward()
			self.optimizer.step()
		else: 
			return train_loss
	return train_loss

def validate(self):
	val_loss = []
	self.net.eval()
	for batch_idx,(voxel_features,voxel_mask,voxel_indices, chamfer_gt) in enumerate(self.val_dataloader):
		zipped_input = self.prepare_gpu_input(voxel_features, voxel_mask, voxel_indices)
		if len(zipped_input) == 2:
			xyz_output = self.customdataparallel(zipped_input)
			#hallucinations = self.customdataparallel(zipped_input) #typing error?
			chamfer_gt = torch.FloatTensor(chamfer_gt)
			loss = self.criterion(xyz_output, Variable(chamfer_gt).cuda(self.gather_device))
			val_loss += [loss.data[0]/chamfer_gt.size(0)]
			if batch_idx % self.args.print_freq == 0:
				#progress_stats = '(val) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.val_dataloader), loss=loss.data[0])
				epoch = 0
				progress_stats = '(val) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.val_dataloader), loss=loss.data[0])
				print(progress_stats)
		else:
			return val_loss
	return val_loss

```



---

- [[코드] DenseLidarNet](https://github.com/345ishaan/DenseLidarNet)

---


main.py의 `#transforms.Lambda(lambda x: logPolar_transform(x)),`를 주석 처리시 문제점 ?