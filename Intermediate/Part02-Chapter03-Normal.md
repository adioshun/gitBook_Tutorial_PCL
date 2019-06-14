# Normal 


Normal은 가장 간단한 특징이라고 볼수 있습니다. 






---

Integral images
Integral images are a method for normal estimation on organized clouds. The algorithm sees the cloud as a depth image, and creates certain rectangular areas over which the normals are computed, by taking into account the relationship between neighboring "pixels" (points), without the need to run lookups on a search structure like a tree. Because of this, it is very efficient and the normals are usually computed in a split second. If you are performing real time computations on clouds taken from a RGB-D sensor, this is the approach you should use because the previous method will probably take several seconds to finish.