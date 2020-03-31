# Euclidean-PCL-Python \(0%\)

## 1. Eucliean Cluster

> 중요 : colorlist is a static variable you should define it outside the function getcolorlist, I defined it after if \_name == ‘\_\_main‘: get\_color\_list.color\_list = \[\]

```python
def do_euclidean_clustering(white_cloud):
    '''
    :param cloud_objects:
    :return: cluster cloud and cluster indices
    '''
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud,cluster_indices

# Euclidean Clustering


white_cloud= XYZRGB_to_XYZ(cloud_objects)
cluster_cloud,cluster_indices = do_euclidean_clustering(white_cloud)
```

Yuchao's blogspot

```python
# Euclidean Clustering
white_cloud = XYZRGB_to_XYZ(white_cloud) # <type 'pcl._pcl.PointCloud'>
tree = white_cloud.make_kdtree() # <type 'pcl._pcl.KdTree'>
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.02) # for hammer
ec.set_MinClusterSize(10)
ec.set_MaxClusterSize(250)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract() # indices for each cluster (a list of lists)
# Assign a color to each cluster
cluster_color = get_color_list(len(cluster_indices))
color_cluster_point_list = []
for j, indices in enumerate(cluster_indices):
for i, indice in enumerate(indices):
color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])
# Create new cloud containing all clusters, each with unique color
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
# publish to cloud
ros_cluster_cloud = pcl_to_ros(cluster_cloud)
# save to local
pcl.save(cluster_cloud, 'cluster.pcd')
```

분리수행

```python
# Euclidean Clustering
def euclid_cluster(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(3000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    return cluster_indices, white_cloud



def cluster_mask(cluster_indices, white_cloud):
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                            white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float( cluster_color[j] )
                                           ])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud


cluster_indices, white_cloud = euclid_cluster(white_cloud)
get_color_list.color_list = []
```

적절한 eps 정하는 방법 : 1. StandardScaler 2. MinMaxScaler

## Region growing segmentation

