# DetectChanges-PCL-Python \(50%\)

> Jupyter 버젼은 [\[이곳\]](https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Intermediate/Part02-Chapter02-DetectChanges-PCL-Python.ipynb)에서 확인 가능 합니다.

```python
cloudA = pcl.PointCloud()
cloudA = pcl.load("RANSAC_plane_false.pcd") 

resolution = 0.01
octree = cloudA.make_octreeChangeDetector(resolution)
octree.add_points_from_input_cloud ()

octree.switchBuffers () #Switch buffers and reset current octree structure.

cloudB = pcl.PointCloud()
cloudB = pcl.load("tabletop_passthrough.pcd")

octree.set_input_cloud (cloudB)
octree.add_points_from_input_cloud ()

newPointIdxVector = octree.get_PointIndicesFromNewVoxels ()

print('Output from getPointIndicesFromNewVoxels:')
cloudB.extract(newPointIdxVector)
```

