#include <pcl/io/pcd_io.h> 
#include <pcl/octree/octree_search.h>  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/point_types.h>  

#include <iostream>
#include <vector>

int main()
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_cluster_0.pcd", *cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}

	//Create an Octree instance object
	float resolution = 0.03f; //Set octree voxel resolution
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); //Create an octree object
	octree.setInputCloud(cloud); 
	octree.addPointsFromInputCloud();  //Build Octree

	//3.1. Voxel Neighbor Search #1
	//pcl::PointXYZRGB searchPoint;
	//searchPoint.x = 0.026256f;
  	//searchPoint.y = -1.464739f;
  	//searchPoint.z = 0.929567f;

	//3.1. Voxel Neighbor Search #2
	pcl::PointXYZRGB searchPoint1 = cloud->points[3000]; //Set the lookup point

	std::vector<int> pointIdxVec;  //Save the result vector of the voxel neighbor search

	if (octree.voxelSearch(searchPoint1, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint1.x
			<< " " << searchPoint1.y
			<< " " << searchPoint1.z << ")"
			<< std::endl;

		//Set the color of the nearest neighbor point
		for (size_t i = 0; i < pointIdxVec.size(); ++i){
			cloud->points[pointIdxVec[i]].r = 255;
			cloud->points[pointIdxVec[i]].g = 0;
			cloud->points[pointIdxVec[i]].b = 0;
		}		
	}

	//3.2.K nearest neighbor search
	pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; 
	int K = 50;
	std::vector<int> pointIdxNKNSearch; //Save the index result of the K nearest neighbor
	std::vector<float> pointNKNSquaredDistance;  //Save the index result of the K nearest neighbor

	std::cout << "K nearest neighbor search at (" << searchPoint2.x
		<< " " << searchPoint2.y
		<< " " << searchPoint2.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{   
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
			cloud->points[pointIdxNKNSearch[i]].r = 0;
			cloud->points[pointIdxNKNSearch[i]].g = 255;
			cloud->points[pointIdxNKNSearch[i]].b = 0;
		}	
	}
	std::cout << "K = 50 nearest neighbors:" << pointIdxNKNSearch.size() << endl;

	//3.3. Neighbor search within radius
	pcl::PointXYZRGB searchPoint3 = cloud->points[1000]; //Set the lookup point
	std::vector<int> pointIdxRadiusSearch;  //Save the index of each neighbor
	std::vector<float> pointRadiusSquaredDistance;  //Save the square of the Euclidean distance between each neighbor and the search point
	float radius = 0.02; //Set the search radius

	std::cout << "Neighbors within radius search at (" << searchPoint3.x
		<< " " << searchPoint3.y
		<< " " << searchPoint3.z
		<< ") with radius=" << radius << std::endl;

	if (octree.radiusSearch(searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{    
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
			cloud->points[pointIdxRadiusSearch[i]].r = 0;
			cloud->points[pointIdxRadiusSearch[i]].g = 0;
			cloud->points[pointIdxRadiusSearch[i]].b = 255;
		}		
	}
	std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << endl;


	pcl::io::savePCDFile<pcl::PointXYZRGB>("Octree_AllinOne.pcd", *cloud);
}

