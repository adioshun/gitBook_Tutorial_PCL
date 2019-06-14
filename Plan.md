# 작성 계획 

- 초급 : 2019. 06월 15일 까지 
- 중급 : 2019. 07월 30일 까지 
- 고급 : 2019. 08월 30일 까지 

```
0% : 작성 예정 
10~50% : 작성 중 
70% : 1차 작성 완료 
80~90% : 보완 중
100% : 작성 완료 
```

---

## 본문 작성 룰 

이미지 크기 : ??



## PCL-Cpp 작성 룰 



> 코드는 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter02-PCL-Cpp.cpp)에서 다운로드 가능합니다. 샘플파일은 [[table_scene_lms400.pcd]](https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/table_scene_lms400.pcd )을 사용하였습니다. 



## Python Jupyter 작성 룰 


```
%load_ext watermark
%watermark -d -v -p pcl,numpy
```

```python 
import tensorflow as tf; print("TensorFLow Version:"+str(tf.__version__))
import keras; print("Keras Version:"+str(keras.__version__))

print "OS:     ", platform.platform()
print "Python: ", sys.version.split("\n")[0]
print "CUDA:   ", subprocess.Popen(["nvcc","--version"], stdout=subprocess.PIPE).communicate()[0].split("\n")[3]
print "LMDB:   ", ".".join([str(i) for i in lmdb.version()])

print caffe.__version__"
!pip freeze | grep pcl 
!printenv
```

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter01-PCL-Python.ipynb)에서 확인 가능 합니다. 


Download `Part02-Chapter03-Normal-PCL-Python.md`로 본문 작성 

주피터 파일명 : `Part02-Chapter03-Normal-PCL-Python.ipynb`

