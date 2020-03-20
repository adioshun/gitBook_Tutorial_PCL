# [Conditional Euclidean Clustering](http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering)

```cpp

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

//Conditional Euclidean Clustering
//http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering


bool
customRegionGrowing (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  if (squared_distance < 10000)
  {
    if (std::abs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (std::abs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (std::abs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}

int
main (int argc, char** argv)
{
  // *.PCD 파일 읽기 
  // https://excellmedia.dl.sourceforge.net/project/pointclouds/PCD%20datasets/Trimble/Outdoor1/Statues_4.pcd.zip
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile <pcl::PointXYZI> ("Statues_4.pcd", *cloud_in);
  // 포인트수 출력
  std::cout << "Loaded :" << cloud_in->points.size () << std::endl;

  // 다운 샘플링 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (80.0, 80.0, 80.0);
  vg.setDownsampleAllData (true);
  vg.filter (*cloud_out);

  // Normal 계산후 합치기 (Set up a Normal Estimation class and merge data in cloud_with_normals)
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
  ne.setInputCloud (cloud_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (300.0);
  ne.compute (*cloud_with_normals);


  // Set up a Conditional Euclidean Clustering class
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
  pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);   //True = 작거나(setMinClusterSize) 큰것도(setMaxClusterSize) 수행 
  cec.setInputCloud (cloud_with_normals);                            // 입력 
  cec.setConditionFunction (&customRegionGrowing);                   // 사용자 정의 조건 
  cec.setClusterTolerance (500.0);                                   //K-NN 탐색시 Radius값 (후보 포인트 탐색에 사용)
  cec.setMinClusterSize (cloud_with_normals->points.size () / 1000); //클러스터 최소 포인트 수 (eg. 전체 포인트 수의 0.1% 이하 ) 
  cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);    //클러스터 최대 포인트 수 (eg. 전체 포인트 수의 20% 이상 )
  cec.segment (*clusters);                                           //군집화 실행 
  
  // True로 초기화시 작거나 큰것의 정보를 저장할 곳 
  pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters);
  pcl::IndicesClustersPtr large_clusters (new pcl::IndicesClusters);
  cec.getRemovedClusters (small_clusters, large_clusters);           


  // Using the intensity channel for lazy visualization of the output
  for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
  for (int i = 0; i < large_clusters->size (); ++i)
    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
  for (int i = 0; i < clusters->size (); ++i)
  {
    int label = rand () % 8;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
  }

  // Save the output point cloud

  pcl::io::savePCDFile ("output.pcd", *cloud_out);


  return (0);
}



/*
bool
enforceIntensitySimilarity (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

bool
enforceCurvatureOrIntensitySimilarity (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}

*/

```
