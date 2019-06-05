The VoxelGrid class implemented by PCL creates a three-dimensional voxel raster from the input point cloud data (which can be thought of as a collection of tiny spatial 3D cubes) and then in each voxel (ie, a three-dimensional cube). The center of gravity of all points in the voxel approximates the other points in the voxel, so that all points in the voxel are finally represented by a center of gravity point, and the filtered point cloud is obtained for all voxels.
--------------------- 
作者：chd_ayj 
来源：CSDN 
原文：https://blog.csdn.net/qq_22170875/article/details/84980996 
版权声明：本文为博主原创文章，转载请附上博文链接！