# 3D Pointcloud Augmentation 


> [Augmentation_Rotation_Jitter.ipynb](https://nbviewer.jupyter.org/github/adioshun/gitBook_Tutorial_PCL/blob/master/Snippets/Augmentation_Rotation_Jitter.ipynb)


## Pointnet2 

```python
    def _augment_batch_data(self, batch_data):
        rotated_data = provider.rotate_point_cloud(batch_data)
        rotated_data = provider.rotate_perturbation_point_cloud(rotated_data)
        jittered_data = provider.random_scale_point_cloud(rotated_data[:,:,0:3])
        jittered_data = provider.shift_point_cloud(jittered_data)
        jittered_data = provider.jitter_point_cloud(jittered_data)
        rotated_data[:,:,0:3] = jittered_data
        return provider.shuffle_points(rotated_data)

# https://github.com/charlesq34/pointnet2/blob/master/utils/provider.py#L32
```


---
https://github.com/hunjung-lim/pcl-tutorial/blob/master/910_Resources/augmentation/augmentation.ipynb
