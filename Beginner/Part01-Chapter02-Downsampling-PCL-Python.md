# PCL-Python 기반 Voxelization

> Jupyter 버젼은 [[이곳]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/Part01-Chapter02-PCL-Python.ipynb)에서 확인 가능 합니다. 


```python 
import pcl

def do_voxel_grid_downssampling(pcl_data,leaf_size):
    '''
    Create a VoxelGrid filter object for a input point cloud
    :param pcl_data: point cloud data subscriber
    :param leaf_size: voxel(or leaf) size
    :return: Voxel grid downsampling on point cloud
    :https://github.com/fouliex/RoboticPerception
    '''
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
    return  vox.filter()
    

cloud = pcl.load("./sample/lobby.pcd")
print(cloud)


LEAF_SIZE = 0.01 
cloud = do_voxel_grid_downssampling(cloud,LEAF_SIZE)
print(cloud)
```
