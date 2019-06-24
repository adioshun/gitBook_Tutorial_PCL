# Point GAN을 이용한 학습 데이터 생성 

> pointGAN : https://github.com/fxia22/pointGAN


## 1. 설치 

```
$ sudo apt-get install python-opencv
$ pip install http://download.pytorch.org/whl/cu80/torch-0.1.11.post5-cp27-none-linux_x86_64.whl 
$ pip install torchvision

$ cd ~
$ git clone https://github.com/fxia22/pointGAN.git
$ cd pointGAN

$ bash build.sh #build C++ code for visualization
$ bash download.sh #download dataset
$ python train_gan.py
$ python show_gan.py --model gan/modelG_10.pth # choose your own model

```



















--- 

Autoencoder for Point Clouds : https://github.com/charlesq34/pointnet-autoencoder 