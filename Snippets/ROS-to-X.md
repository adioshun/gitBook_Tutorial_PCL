# ROS Pointcloud2 데이터를 numpy(np)파일로 저장 

> https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/didi_data/ros_scripts/run_dump_lidar.py
> 개인적으로 pickle 파일 저장 추천


* [ROS bags-TO-Image.ipynb](https://gist.github.com/anonymous/4857f8920c9fc901121a429ead32a7db)
* [ROS bags-TO-Point Clods.ipynb](https://gist.github.com/anonymous/e675ea14113252be321320be62248034)
* [ROS bags-TO-Avi.ipynb](https://gist.github.com/anonymous/fb1e98efe187b2a35b6d91fb5df9e83b)










## 2. ROS-to-PCD

bag파일에서 pcd파일을 추출 하시려면 아래의 명령어를 이용 하면 됩니다. 

```
$ rosrun pcl_ros bag_to_pcd lobby_lidar.bag /velodyne_points ./lobby_pcd
# rosrun pcl_ros bag_to_pcd <input_file.bag> / <output_directory> 
```

또는 

```python 

#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import cv2
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

save_path = None

def cloud_loader(msg):
    timestamp = msg.header.stamp.secs + ((msg.header.stamp.nsecs + 0.0) / 1000000000)
    save_pcd(msg, timestamp, save_path)

def save_pcd(cloud, timestamp, path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)), dtype=np.float32)[:, 0:3])
    p.to_file(path + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')

def rosbag_data_extract_sample():
    global save_path
    try:
        save_path = sys.argv[1]
        topic = sys.argv[2]
    except Exception, e:
        #sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")
        save_path = './sample'

    node_name = "get_%s_and_convert_to_PCD_data" % topic
    rospy.init_node('rosbag_pcd_extract_unsync', anonymous=True)

    rospy.Subscriber(topic, PointCloud2, cloud_loader)
    rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract_sample()

```

---

### 1.1 pickle로 생성 

```python 

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rosbag
import os 
import pickle
import argparse


# ROSBAG SETTINGS
#data_dir = "/workspace/_rosbag/"
#bag_name = 'cafeteria_baseline_2018-11-16-03-24-16.bag'
#bag_file = os.path.join(data_dir, bag_name)


parser = argparse.ArgumentParser()
parser.add_argument("--baseline", help="Use baseline rosbag to remove background")

args = parser.parse_args()

bag_file = args.baseline



bag = rosbag.Bag(bag_file, "r")
print("")
print("Load rosbag : {}".format(bag_file))

messages = bag.read_messages(topics=['/velodyne_points'])
n_lidar = bag.get_message_count(topic_filters=["/velodyne_points"])


baseline = np.zeros((0,5),dtype=np.float32)




for i in range(50):#n_lidar):
    # READ NEXT MESSAGE IN BAG
    topic, msg, t = messages.next()  #print(dir(msg))

    # CONVERT MESSAGE TO A NUMPY ARRAY OF POINT CLOUDS
    # creates a Nx5 array: [x, y, z, reflectance, ring]
    lidar = pc2.read_points(msg)
    lidar = np.array(list(lidar),  dtype=np.float32)
    baseline = np.concatenate((baseline, lidar),axis=0)
    print("ID : {}-{}".format(i, baseline.shape[0]))

baseline = baseline[:,0:3]
#save_file = bag_name + ".npy"
#np.save(save_file, lidar)

save_file = bag_file + ".pkl"
output = open(save_file, 'wb')
pickle.dump(baseline, output)
output.close()

print("")
print("File saved : {}".format(save_file))
print("")


```

