---
[색상 변경 예제 추가]

```python 
float_rgb = pcl_helper.rgb_to_float([255,255,255])

points_list = []
for data in cloud_arr:
        points_list.append([data[0], data[1], data[2], float_rgb])

cloud = pcl.PointCloud_PointXYZRGB()
cloud.from_list(points_list)
```