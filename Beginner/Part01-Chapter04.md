# Noise Filtering

Lidar는 센서 특성상 물체가 존재 하지 않아도 먼지등으로 인해 point가 생성 됩니다. 이렇게 생성된 point들을 Noise로 간주 하고 제거 하는 작업을 진행 해야 합니다.

다행히 이러한 노이즈들은 정상적인 point 대비 빈 공간에 소수의 점들만 탐지 되므로 이러한 특성을 이용하여 제거 할수 있습니다.

- Statistical based

- Radius based


> **[중요]** 현재 Radius based 방식은 정상 동작 하지 않는다고 합니다. 파라미터를 바꾸어도 결과가 '0'이라고 하네요. [[참고]](https://github.com/strawlab/python-pcl/issues/211) - 2018.06.11



The StatisticalOutlierRemoval filter can be used primarily to eliminate outliers or to measure the gross errors caused by errors . 
The filtering idea is: perform a statistical analysis on the neighborhood of each point and calculate its average distance to all adjacent points. Assume that the result is a Gaussian distribution whose shape is determined by the mean and standard deviation, then the point beyond the standard range (defined by the global distance mean and variance) can be defined as the outlier and from the data. Removed.
--------------------- 
作者：chd_ayj 
来源：CSDN 
原文：https://blog.csdn.net/qq_22170875/article/details/84994029 
版权声明：本文为博主原创文章，转载请附上博文链接！





|![](https://img-blog.csdnimg.cn/20190412162257278.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)|![](https://img-blog.csdnimg.cn/20190412162346938.PNG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)|
|-|-|