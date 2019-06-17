# PCL-Python을 이용한 분류 

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter03-PCL-Python-a.ipynb)에서 확인 가능 합니다. 





```python
#!/usr/bin/env python
import pickle
import itertools
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.preprocessing import LabelEncoder, StandardScaler
```

# [features.py](https://github.com/mkhuthir/RoboND-Perception-Project/blob/master/src/sensor_stick/src/sensor_stick/features.py)


```python
import matplotlib.colors

def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

def compute_color_histograms_PCD(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    """
    # Step through each point in the point cloud for ROS msg
    for point in pc2.read_points(cloud, skip_nans=True): 
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)
    """
    
    # Step through each point in the point cloud for PCD
    for point in cloud[:,3]: # for PCD file
        rgb_list = float_to_rgb(point)
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # Compute histograms
    nbins=32
    bins_range=(0, 256)
        
    # Compute the histogram of the channels separately
    channel_1_hist = np.histogram(channel_1_vals, bins=nbins, range=bins_range)
    channel_2_hist = np.histogram(channel_2_vals, bins=nbins, range=bins_range)
    channel_3_hist = np.histogram(channel_3_vals, bins=nbins, range=bins_range)
    
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel_1_hist[0], channel_2_hist[0], channel_1_hist[0])).astype(np.float64)
    
    # Normalize the result
    normed_features = hist_features / np.sum(hist_features)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96) 

    # Return the feature vector
    return normed_features 
```


```python

def get_normals(cloud_path):
    """
    The actual *compute* call from the NormalEstimation class does nothing internally but:
    for each point p in cloud P
        1. get the nearest neighbors of p
        2. compute the surface normal n of p
        3. check if n is consistently oriented towards the viewpoint and flip otherwise

    # normals: pcl._pcl.PointCloud_Normal,size: 26475
    # cloud: pcl._pcl.PointCloud
    """
    cloud = pcl.load(cloud_path)
    
    feature = cloud.make_NormalEstimation()
    #feature.set_RadiusSearch(0.1) #Use all neighbors in a sphere of radius 3cm
    
    feature.set_KSearch(3)
    normals = feature.compute()
    
    return normals

def compute_normal_histograms(normal_cloud, nbins=32, nrange=(-1,1)):
    '''
    Computes and bins the point-cloud data using the objects distribution of surface normals.
    :param: normal_cloud, point cloud containing the filtered clusters.
    :param: nbins,number of bins that data will be pooled into.
    :param: nrange, value range of the data to be pooled.
    :return: the normalised histogram of surface normals
    '''
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for I in range(0,normal_cloud.size):
        norm_x_vals.append(normal_cloud[I][0])
        norm_y_vals.append(normal_cloud[I][1])
        norm_z_vals.append(normal_cloud[I][2])

    # Compute histograms of normal values (just like with color)
    norm_x_hist = np.histogram(norm_x_vals, bins=nbins, range=nrange)
    norm_y_hist = np.histogram(norm_y_vals, bins=nbins, range=nrange)
    norm_z_hist = np.histogram(norm_z_vals, bins=nbins, range=nrange) 

    # Concatenate and normalize the histograms
    hist_features = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])).astype(np.float64)
    normed_features = hist_features / np.sum(hist_features)

    return normed_features
```

# 예측 


```python
cloud = pcl.load_XYZRGB("tabletop.pcd")
sample_cloud = cloud.to_array()
```


```python
# Generate Color Histogram for the spawned model
# Enable using_hsv for better results
c_hists = compute_color_histograms_PCD(sample_cloud, using_hsv=True)

# Generate normals and notmal histograms for the spawned model
normals = get_normals("tabletop.pcd")
n_hists = compute_normal_histograms(normals)

# Generate feature by concatenate of color and normals.
feature = np.concatenate((c_hists, n_hists))
```


```python
detected_objects = []
# Make the prediction, retrieve the label for the result
# and add it to detected_objects_labels list
################################
model = pickle.load(open('model.sav', 'rb'))
#https://raw.githubusercontent.com/mkhuthir/RoboND-Perception-Project/master/model.sav

clf = model['classifier']
encoder = LabelEncoder()
encoder.classes_ = model['classes']
scaler = model['scaler']

prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
label = encoder.inverse_transform(prediction)[0]
print("Predicted Result : ", label)
#모든 결과를 Soap로 인식, 수정 필요 
```

    ('Predicted Result : ', 'soap')





---

https://github.com/camisatx/RoboticsND/blob/master/projects/perception/README.md#object-recognition


https://github.com/georgeerol/RoboticPerception#object-recognition

https://hortovanyi.wordpress.com/2017/11/19/3d-perception-project/


https://github.com/dexter800/RoboND-Perception-Project

https://github.com/dexter800/RoboND-Perception-Project/blob/master/project.py.py