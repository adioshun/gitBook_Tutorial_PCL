# Kd-Tree/Octree Search

## 1. Kd-tree/Octree를 이용한 점군 탐색 및 배경 제거 

특정 점을 기준으로 가까이 있는 점들은 무엇인지? 좌표는 어떻게 되는지 알고 싶을 때가 있습니다\(efficiently query for objects at or near a location\). 이러한 탐색들은 점군에서 이웃 점 들과의 비교를 통한 특징 추출에 큰 도움이 됩니다. 이전에 살펴보았던 filters외에도 향후, surface, features, registration등도 탐색 기능에 바탕을 두고 있습니다. PCL에서는 다양한 탐색 방법을 제공하고 있습니다.

* BruteForce :simple brute force search algorithm 
* OrganizedNeighbor : KNN search in organized point clouds 
* KdTree 
* Octree 
* FlannSearch 

가장 간단한 방법은 모든 데이터의 위치 정보를 array에 저장하고, 전체 데이터를 탐색해서 찾을 수도 있습니다. 하지만 매 순간 매 포인트에 대하여 검색할 경우 시간이 많이 걸립니다. 따라서 공간을 분할하여 구조화하여 탐색을 하면 좋습니다. Spatial partitioning 효율적으로 수행하기 위해 space-partitioning data structure를 생성하면 됩니다. spatial data structure는 위치 정보를 저장하고 있는 데이터 구조체입니다. 이 형태로 데이터터를 저장하게 되면 효율적으로 레인지 서치와 근접 이웃 탐색이 가능 합니다.

![\[&#xADF8;&#xB9BC; 1\] Grid, Quadtree, Octree, k-d Tree](https://user-images.githubusercontent.com/17797922/106234240-bee1e580-623b-11eb-9365-89ea6738cacd.png)

가장 간단한 spatial partitions 방법은 앞에서 배운 그리드\(2D\)/복셀\(3D\)을 이용하는 것입니다. 격자를 통한 접근법은 골고루 분산되어 있는 점에 대해서 구현하기 쉬운 해법이지만, 적절한 크기를 선정하는 방법과 빈공간으로 인한 메모리 낭비와 계산 효율성이 떨어 집니다. 공간을 조금 더 적절히 나누는 방법은 공간분할 트리\(Space-partitioning trees\)를 사용하는 것입니다. 

대표적인 방법은 아래와 같습니다. 

* k-d trees 
* Octrees\(3D\)

### 1.1. Kd-TRee기반 검색 

BST\(Binary Search Tree\)를 다차원 공간으로 확장한 것으로, 각 노드의 데이터가 공간의 K 차원 포인트인 이진 검색 트리입니다. 기본 구조와 알고리즘은 BST와 유사하지만 트리의 레벨 차원을 번갈아 가며 비교한다는 점이 다릅니다. \[그림 2\]는 BST와 2D, 3D kd-tree 를 표현한 것입니다.

![\[&#xADF8;&#xB9BC; 2\] 2D, 3D, Kd-tree ](https://user-images.githubusercontent.com/17797922/106234662-91496c00-623c-11eb-8d7f-84b2a734dbe0.png)

