#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
std::cout << "Loaded " << cloud->width * cloud->height << std::endl;

// Create the filtering object
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (cloud);
sor.setMeanK (50);
sor.setStddevMulThresh (1.0);
sor.filter (*cloud_filtered);

std::cout << "Filtered " << cloud_filtered->width * cloud_filtered->height << std::endl;
pcl::io::savePCDFile<pcl::PointXYZ>("StatisticalOutlierRemoval.pcd", *cloud_filtered);

sor.setNegative (true);
sor.filter (*cloud_filtered);
pcl::io::savePCDFile<pcl::PointXYZ>("StatisticalOutlierRemoval_Neg.pcd", *cloud_filtered);

return (0);
}
