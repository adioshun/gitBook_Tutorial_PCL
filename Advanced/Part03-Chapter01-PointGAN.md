# Point GAN을 이용한 학습 데이터 생성 

> pointGAN : https://github.com/fxia22/pointGAN

> https://github.com/hunjung-lim/awesome-vehicle-datasets/blob/master/Object/shpaenet/README.md

## 1. 설치 

```
$ sudo apt-get install python3-opencv python3-tk
$ pip3 install http://download.pytorch.org/whl/cu80/torch-0.1.11.post5-cp27-none-linux_x86_64.whl 
$ pip3 install torchvision progressbar2


$ cd ~
$ git clone https://github.com/fxia22/pointGAN.git
$ cd pointGAN

$ bash build.sh #build C++ code for visualization
$ bash download.sh #download dataset
```



## 2. 학습 


### 2.1 학습에 사용할 테이터를 포맷 맞게 저장 


### 2.2 

```
vi synsetoffset2category.txt 
human 777777
```

`train_gan.py` L136주석처리 
```
# print('[%d: %d/%d] train lossD: %f lossG: %f' %(epoch, i, num_batch, lossD.data[0], lossG.data[0]))
$ python3 show_gan.py --model gan/modelG_10.pth # choose your own model


```




## 3. 생성 

```
python3 show_gan.py --model gan/modelG_3.pth
#gen = PointGen(num_points = 2500)#2048) 
```














--- 

# Autoencoder for Point Clouds을 이용한 학습 데이터 생성 (0%)

> https://github.com/charlesq34/pointnet-autoencoder 