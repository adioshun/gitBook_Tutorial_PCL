#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

int
main(int argc, char** argv)
{
// Objects for storing the point clouds.
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

// Read a PCD file from disk.
pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud);
std::cout << "Loaded " << cloud->width * cloud->height << std::endl;

// Filtering object.
pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
filter.setInputCloud(cloud);
// Object for searching.
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
filter.setSearchMethod(kdtree);
// Use all neighbors in a radius of 3cm.
filter.setSearchRadius(0.03);
// Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
// and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
// Radius around each point, where the local plane will be sampled.
filter.setUpsamplingRadius(0.03);
// Sampling step size. Bigger values will yield less (if any) new points.
filter.setUpsamplingStepSize(0.02);

filter.process(*filteredCloud);

pcl::io::savePCDFile<pcl::PointXYZ>("table_scene_lms400_upsampled.pcd", *filteredCloud);
std::cout << "Result " << filteredCloud->width * filteredCloud->height << std::endl;
}