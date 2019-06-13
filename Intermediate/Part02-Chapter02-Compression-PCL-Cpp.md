# PCL-Cpp 기반 압축 

```cpp

#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	pcl::io::loadPCDFile<pcl::PointXYZ> ("lobby.pcd", *cloud);
	


	// Octree compressor object.
	// Check /usr/include/pcl-<version>/pcl/compression/compression_profiles.h for more profiles.
	// The second parameter enables the output of information.
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
	// Stringstream that will hold the compressed cloud.
	std::stringstream compressedData;

	// Compress the cloud (you would save the stream to disk).
	octreeCompression.encodePointCloud(cloud, compressedData);

	// Decompress the cloud.
	octreeCompression.decodePointCloud(compressedData, decompressedCloud);
	


}
```