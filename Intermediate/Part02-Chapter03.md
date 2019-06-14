# 특징(Feature) 찾기 

특징이란 각 포인트들이 가진 고유 성질로 각 포인트들을 구분 할때 사용 됩니다.  

2D 이미지 분석을 다루어 보신 분이라면 특징점(Keypoint/Feature)와 특징 기술자(Feature descriptor)라는 용어에 대하여 아실것입니다. 

특징점이란 그 image의 특징을 잘 나타내줄 수 있는 부분을 의미 합니다. 대표적으로 다각형의 꼭지점(corner)'이나 '선분의 끝점가 있습니다. 특징점은 물체 탐지, 물체 추적, 물체 매칭등에 사용됩니다. 특징점 추출을 위해서  Harris, SIFT, FAST 알고리즘들 있습니다. 

특징 기술자는 특징점의 지역적 특성을 설명합니다. 따라서 특징점간 비교가 가능해 집니다. 대표적인 특징 기술자는 SIFT, HOG 등이 있습니다. 

3D 포인트 클라우드 분석시에도 이러한 특징(Feature)정보들을 활용 합니다. 다음 챕터에서 다룰 분류문제 해결을 위해서는 필수 적입니다. 

![image](https://user-images.githubusercontent.com/17797922/47074467-68e8ff80-d235-11e8-9c5c-541cf31ac671.png)

자세한 내용은 [[이곳]](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_\(descriptors\))에 잘 기술 되어 있습니다. 




