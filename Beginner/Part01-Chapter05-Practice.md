# 



```python 

def plane_filter(input_pcl_xyz, input_max_distance):
    max_distance = input_max_distance #0.01
    ransac_segmentation = filter.do_ransac_plane_segmentation(input_pcl_xyz,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,max_distance)
    #Extract inliers and outliers
    plane,objects = filter.extract_cloud_objects_and_cloud_table(input_pcl_xyz,ransac_segmentation )

    return objects

def plane_filter_normal(input_pcl_xyz, input_max_distance):
    seg = input_pcl_xyz.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    seg.set_max_iterations(100)
    seg.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, model = seg.segment()
    objects = input_pcl_xyz.extract(indices, negative=True)

    return object
    


    roi_plane_normal_pclxyz = plane_filter_normal(roi_pcl_xyzrgb, 0.05)  # 바닥 제거(RANSAC) 
    roi_plane_normal_ros_msg = pcl_helper.pcl_to_ros(roi_plane_normal_pclxyz) #PCL을 ROS 메시지로 변경 
    pub = rospy.Publisher("/velodyne_roi_plane_normal", PointCloud2, queue_size=1)
    pub.publish(roi_plane_normal_ros_msg)

---

## with ROS

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
## http://wiki.ros.org/pcl/Tutorials



ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::fromROSMsg (*input, cloud);

pcl::ModelCoefficients coefficients;
pcl::PointIndices inliers;
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;
// Optional
seg.setOptimizeCoefficients (true);
// Mandatory
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (0.01);

seg.setInputCloud (cloud.makeShared ());
seg.segment (inliers, coefficients);

// Publish the model coefficients
pcl_msgs::ModelCoefficients ros_coefficients;
pcl_conversions::fromPCL(coefficients, ros_coefficients);
pub.publish (ros_coefficients);
}

int
main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");
ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
#pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

// Spin
ros::spin ();
}
```