# Min-Cut Based Segmentation

> Aleksey Golovinskiy, Thomas Funkhouser, [Min-Cut Based Segmentation of Point Clouds](https://gfx.cs.princeton.edu/pubs/Golovinskiy_2009_MBS/index.php), ICCV 2009

In this tutorial we will learn how to use the min-cut based segmentation algorithm implemented in the  `pcl::MinCutSegmentation`  class. 

This algorithm makes a binary segmentation of the given input cloud. 

Having objects center and its radius the algorithm divides the cloud on two sets: 
- foreground
- background points 
- (points that belong to the object and those that do not belong).

# Theoretical Primer

The idea of this algorithm is as follows:

#### 1.  For the given point cloud algorithm constructs the graph that contains every single point of the cloud as a set of vertices and two more vertices called source and sink. 
- Every vertex of the graph that corresponds to the point is connected with source and sink with the edges. 
- In addition to these, every vertex (except source and sink) has edges that connect the corresponding point with its nearest neighbours.
    
#### 2.  Algorithm assigns weights for every edge. There are three different types of weight. Let’s examine them:

######  First of all it assigns weight to the edges between clouds points. This weight is called smooth cost and is calculated by the formula:
    
![smoothCost=e^{-(\frac{dist}{ \sigma })^2}](http://pointclouds.org/documentation/tutorials/_images/math/1fc3ab04b431a5a9fc94ff920fea881f359e93ad.png)

- Here  `dist`  is the distance between points. The farther away the points are, the more is probability that the edge will be cut.
    
######  Next step the algorithm sets data cost. 
- It consists of foreground and background penalties. 
- The first one is the weight for those edges that connect clouds points with the source vertex and has the constant user-defined value. 
- The second one is assigned to the edges that connect points with the sink vertex and is calculated by the formula:

![backgroundPenalty=(\frac{distanceToCenter}{radius})](http://pointclouds.org/documentation/tutorials/_images/math/cb263f17e35975b9e36257b7ca48ee6c4ae4109d.png)    

- Here  `distanceToCenter` is the distance to the expected center of the object in the horizontal plane:
    
![distanceToCenter=\sqrt{(x-centerX)^2+(y-centerY)^2}](http://pointclouds.org/documentation/tutorials/_images/math/5a41b013d617316f08a1ed173341ad112945652e.png)
    
Radius that occurs in the formula is the input parameter for this algorithm and can be roughly considered as the range from objects center outside of which there are no points that belong to foreground (objects horizontal radius).

####  3. After all the preparations the search of the minimum cut is made. Based on an analysis of this cut, cloud is divided on foreground and background points.

