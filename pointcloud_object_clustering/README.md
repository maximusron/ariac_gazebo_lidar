# PointCloud Object Clustering

## Clustering Process:

All of the below steps can be controlled by changing the parameters in `config/params.yaml`.

1. Ground segmentation
	- Detecting and removing the points related to the ground plane using the `SACMODEL_PERPENDICULAR_PLANE` model with the `RANSAC` method.

2. Downsample 
	- Downsampling the point cloud using `VoxelGrid`.

3. ROI filtering
	- Given `x_min` `x_max` `y_min` `y_max` `z_min` `z_max` a pass through filter is used to remove points outside of the given boundaries. If lane equations are being given the y intercepts are used along with a pass through filter to only consider the points in between the two y intercepts. Along with that only points in front of the car are considered from `y_dist_car` upto `y_max`.

4. Difference of Normals segmentation
	- `DifferenceOfNormalsEstimation` is used. `radius_small` and `radius_large` are each used to find an estimated normal of each point. Then for each point the normalized difference of normals is calculated. This is later used to filter the points with the given `threshold` (DoN L2 norm, filtering points by scale membership, large magnitudes indicates point has a strong response at the given scale parameters).

5. Clustering
	- Clusters are found in the filtered point cloud using `EuclideanClusterExtraction`. After one or more clusters are found, whatever point is closest to the robot with respect to the y axis is published.

6. Finding Centroids and bounding boxes
	- First a min max bounding box is fit to a cluster then using the vertices of the box the centroid of the box is found NOT of the surface. `BoundingBoxes_centroids_vertices`.


## ROS

#### TF Frame
- `tf_frame_id` The frame to which pcl-ros converts the point cloud using a transform listener.

#### Publisher
- `CENTROID_VERTICES_PUB` Publishes the centroids and the vertices of a bounding box. (for visualisation purposes)
- `CENTROIDS_PUB` Publishes the centroids (sensor_msgs::PointCloud2). (for visualisation purposes)
- `BOUNDING_BOX_PUB` Publishes bounding boxes (visualization_msgs::Marker).

#### Subscriber
- `point_cloud_topic` Gets the unfiltered raw point cloud (sensor_msgs::PointCloud2).

## Parameters 
- `MAX_HEIGHT` (0.3) maximum height of the ground plane
- `FLOOR_MAX_ANGLE` (0.18) maximum angular deviation while fitting a plane to the ground (RADIANS)

- `LEAF_SIZE` (0.2) leaf size for downsampling (voxel grid)

- `MAX_CLUSTER_SIZE` (500) maximum points allowed in a cluster
- `MIN_CLUSTER_SIZE` (30) minimum required points in a cluster
- `CLUSTER_TOLERANCE` (0.5) distance tolerance for points to be considered part of a cluster

 ROI - Region in which the point cloud particles are considered for clustering
- `x_min` 
- `x_max` 
- `y_min` 
- `y_max` 
- `z_min` 
- `z_max`  
  
  In factory floors, the general FOV ranges from 3 to 40 cm, use ```z_min = 0.03``` , ```z_max = 0.4 ```

- `radius_small` (0.5) for DoN segmentation
- `radius_large` (2.0) for DoN segmentation
- `threshold` (0.5) DoN L2 norm threshold value

- `bool_filter_ground` (true) boolean value
- `bool_downsample` (true) boolean value
- `bool_ROI_filtering` (true) boolean value
- `bool_difference_normals_segmentation` (false) boolean value
- `bool_find_centroids_and_vertices` (true) Centroids and vertices of bounding boxes in the format:
  
[Centroid1X, Centroid1Y, Centroid1Z,
min_point1.x, min_point1.y, min_point1.z,
max_point1.x, min_point1.y, min_point1.z,
max_point1.x, min_point1.y, max_point1.z,
min_point1.x, min_point1.y, max_point1.z,
min_point1.x, max_point1.y, max_point1.z,
max_point1.x, max_point1.y, max_point1.z,
max_point1.x, max_point1.y, min_point1.z,
min_point1.x, max_point1.y, min_point1.z .... Centroid2X....]




