# 인식-GeometricConsistencyGrouping

```cpp
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
 
typedef pcl::PointXYZ PointType; 
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;

int main()
{

  // 3D Object Recognition based on Correspondence Grouping
  // http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
  // 

  // PCL/OpenNI tutorial 5: 3D object recognition (pipeline)
  // http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>()); 
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
 
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
 

  // 파일 읽기 
  // https://github.com/PointCloudLibrary/pcl/blob/master/test/milk.pcd?raw=true
  // https://github.com/PointCloudLibrary/pcl/blob/master/test/milk_cartoon_all_small_clorox.pcd?raw=true
	pcl::io::loadPCDFile("milk.pcd", *model);
	pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *scene);

 
	// 노멀 계산 Compute Normals
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setNumberOfThreads(4); 
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals); 
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);
 
	// 다운 샘플링 
  // 샘플링 결과를 키포인트로 사용하여 3D descriptor생성 
  // 최종적으로 key-point matching과 determining point-to-point correspondence에 활용 

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(0.0114425);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

 	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(0.0305134);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
	
	// SHOT 기술자 생성 
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(0.0228851);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);
 
	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);
 	
	//KdTree 탐색을 이용하여 Correspondences 유사도 계산 
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences()); //결과 저장 
	pcl::KdTreeFLANN<DescriptorType> match_search; 
	match_search.setInputCloud(model_descriptors);
 	// 각 scene keypoint descriptor와 model keypoints descriptor 간의 nearest neighbor 검색 
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  유사도 구분 기준 유클리드 거리 0.25 
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
 


	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (0.01f);
    gc_clusterer.setGCThreshold (5.0f);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);


	//  Output results
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
 
		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
 
		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
 


	//  시각화 
  bool show_keypoints_(false);
  bool show_correspondences_(true);


	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");
 
	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
 
 
	if (show_correspondences_ || show_keypoints_)
	{
		  //We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}
 
	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}
 
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
 
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
 
		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
 
				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
 
	system("pause");
	return (0);
}

```

