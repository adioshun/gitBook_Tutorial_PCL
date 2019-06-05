The StatisticalOutlierRemoval filter can be used primarily to eliminate outliers or to measure the gross errors caused by errors . 
The filtering idea is: perform a statistical analysis on the neighborhood of each point and calculate its average distance to all adjacent points. Assume that the result is a Gaussian distribution whose shape is determined by the mean and standard deviation, then the point beyond the standard range (defined by the global distance mean and variance) can be defined as the outlier and from the data. Removed.
--------------------- 
作者：chd_ayj 
来源：CSDN 
原文：https://blog.csdn.net/qq_22170875/article/details/84994029 
版权声明：本文为博主原创文章，转载请附上博文链接！





|![](https://img-blog.csdnimg.cn/20190412162257278.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)|![](https://img-blog.csdnimg.cn/20190412162346938.PNG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)|
|-|-|