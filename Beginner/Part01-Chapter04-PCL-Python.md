# PCL-Python 기반 노이즈 제거 


## 1. Statistical Outlier Removal

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter04-PCL-Python.ipynb)에서 확인 가능 합니다. 



```python 

def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point
    :param tresh:   Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

    
cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    
```

    입력 cloud포맷 : pcl_xyz 
    pcl_xyz = pcl_helper.XYZRGB_to_XYZ(pcl_xyzrgb)
    pcl_xyzrgb시 : TypeError: __cinit__() takes exactly 1 positional argument (0 given) 에러 

First filter is the PCL’s Statistical Outlier Removal filter. in this filter for each point in the point cloud, it computes the distance to all of its neighbors, and then calculates a mean distance. By assuming a Gaussian distribution, all points whose mean distances are outside of an interval defined by the global distances mean+standard deviation are considered to be outliers and removed from the point cloud.

Mean K = 3 was the best value I found to almost remove all noise pixels. any value higher than 3 was leaving some of the noise pixels behind. x was selected to be 0.00001.

---
## 2. Radious Outlier Removal



> [추후 추가](https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/remove_outliers.py)