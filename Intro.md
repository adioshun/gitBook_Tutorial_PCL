# 개요 


새로운 프로젝트를 진행 하면서 Point cloud Library(PCL)를 처음 접하게 되었습니다. 3D 데이터의 처리를 위해서는 필수적인 Library인데 개발자 홈페이지를 제외 하고는 정리 되어 있는 문서도 질문이나 의견을 교류 할 수 있는 곳도 적어 개념을 이해하거나 적용 하는데 많은 어려움이 있었습니다. 프로젝트 1년차가 종료 되어 가는 시점에서 그동안 익혔던 내용 및 코드 정리도 하고, 이 분야를 시작하시려는 분들에게 조금이나마 도움이 되고자 Tutorial을 작성 하게 되었습니다. 


- 기본 내용은 PCL 홈페이지의 [[Documentation-Tutorials]](http://pointclouds.org/documentation/tutorials/)와 Eugen Cicvarić의 [[3D Object Recognition and Pose Estimation using Point Cloud Library]](https://drive.google.com/file/d/1QtQTlm3_FiOdBslbtMAubVMyd2Bjofl1/view?fbclid=IwAR0NZfTAvfSwg_X_Flx5Uhg5GMLRaNFdgKU6PZRsHuskc95Sd2ErAKLg4LM), [[wikipedia]](https://www.wikipedia.org/)를 중심으로 하였습니다. 

- 그 외 참고한 여러 자료들은 *[References]*페이지에 별도 업데이트 하도록 하겠습니다. 

- PCL에 대한 정보 공유나 궁금한점은 **[[페이스북 PCL Research Group KR]](https://www.facebook.com/groups/165198587522918/)**에 올려 주세요. 

- Tutorial은 [초안(Gitbook)](https://adioshun.gitbooks.io/pcl-tutorial/content/), [백업(Github)](https://github.com/adioshun/gitBook_Tutorial_PCL), [최종본(Wikidocs)](https://wikidocs.net/book/827)에 동시 저장되어 있습니다. 



## 


## PCL이란? 


PCL은 `Point cloud Library`의 약어로 Lidar나 RGB-D센서 등으로 수집되는 점군(Point cloud)를 처리 하기 위한 라이브러리 입니다.

Point cloud를 처리를 위한 라이브러리로는 PCL, PCL-python, Open3D, pyPCD, Laspy, PCLpy 등이 있습니다.

여기서는 PCL-Python과 일부 Open3D를 활용합니다.


---

# 문서 구성 및 내용 

### 이론 


### 실습 




PCL은 `Point cloud Library`의 약어로 Lidar나 RGB-D센서 등으로 수집되는 점군(Point cloud)를 처리 하기 위한 라이브러리 입니다.

Point cloud를 처리를 위한 라이브러리로는 PCL, PCL-python, Open3D, pyPCD, Laspy, PCLpy 등이 있습니다.

여기서는 PCL-Python과 일부 Open3D를 활용합니다.

진행은 Lidar로 수집되는 점군 데이터에서 사람을 추출 하는 3D People Detection 구현을 목표로 하고 있습니다.




# 선지식 


---


# 작성 계획 및 목차 

|            | 초급                | 중급                   | 고급               |
|:----------:|---------------------|------------------------|--------------------|
| 2018.11.22 | 0.1 ~~Home/PCL~~    |                        |                    |
| 2018.11.22 | 0.2 환경구축        |                        |                    |
| 2018.11.23 | 1.1 Down Sampling   |                        |                    |
| 2018.11.23 | 1.2 ROI Filtering   |                        |                    |
| 2018.11.29 | 1.3 Noise Filtering |                        |                    |
| 2018.11.29 | 1.4 Plane Removal   |                        |                    |
| 2018.11.30 | 1.5 Clustering      |                        |                    |
| 2018.12.03 |                     | 2.1 Background Removal |                    |
| 2018.12.04 |                     | 2.2 Clustering         |                    |
| 2018.12.05 |                     | 2.3 Tracking           |                    |
| 2019.03.01 |                     |                        | 3.1 Clustering     |
| 2019.04.01 |                     |                        | 3.2 Classification |
| 2019.05.01 |                     |                        | 3.3 Tracking       |
