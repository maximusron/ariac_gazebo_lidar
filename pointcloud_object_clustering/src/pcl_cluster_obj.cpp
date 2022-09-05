#include "pcl_cluster_obj.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

void PCLClusterObject::Downsampling()
{
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (modified_cloud);
	sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter (*modified_cloud);
}

void PCLClusterObject::DifferenceOfNormalsSegmentation()
{
	// Creating a search tree to calculate normals
	pcl::search::Search<pcl::PointXYZ>::Ptr tree;
	if (modified_cloud->isOrganized())
	{
		//OrganizedNeighbor search used for organized data
		std::cout<<"Organized";
		tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
	}
	else
	{
		// KdTree is used for unorganized data
		std::cout<<"Unorganized";
		tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
	}

	// setting input for search tree
	tree->setInputCloud(modified_cloud);

	// init normal estimation
	// NormalEstimationOMP estimates surface normalsn in parallel using OpenMP (this supports multiprocessing)
	// for GPU accelerated use: pcl::gpu::NormalEstimation
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne; // <input_type, output_type>
	ne.setInputCloud(modified_cloud);
	ne.setSearchMethod(tree);
	// setting an arbitary view point for consistent orientation
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
					std::numeric_limits<float>::max());

	// calculating normals with radius_small
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch (radius_small);
	ne.compute (*normals_small_scale);

	// calculating normals with radius_large
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch (radius_large);
	ne.compute (*normals_large_scale);

	// creating an uninitialized vector field for the point cloud
	// since the restul of difference of normals is a vector field
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal> (*modified_cloud, *don_cloud);

	// creating DoN operator
	// <input_type, estimated_normal_type, vector_field_output_type>
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(modified_cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if (!don.initCompute())
	{
		std::cout << "Error initializing DoN" << std::endl;
	}

	// compute DoN
  	// don.initCompute();
	don.computeFeature(*don_cloud);

	// DoN based filtering
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionOr<pcl::PointNormal>());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
	                           new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
	);
	// Build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(don_cloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	condrem.filter (*doncloud_filtered);

	// stroing in pcl::pointXYZ for further processing / clustering
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*doncloud_filtered, *modified_cloud);
}

void PCLClusterObject::RemoveGroundPlane()
{
	pcl::SACSegmentation<pcl::PointXYZ> seg; //segmentation object
	pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices); //to store the indices of the inliers
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //coeffs in the form ax+by+cz+d=0

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // want to find a perpendicular plane to the z axis
	seg.setMethodType(pcl::SAC_RANSAC); // RANSAC robust estimator
	seg.setMaxIterations(100);
	seg.setAxis(Eigen::Vector3f(0, 0, 1)); // setting z axis
	seg.setEpsAngle(FLOOR_MAX_ANGLE); // max angular deviation (RADIANS)

	seg.setDistanceThreshold(MAX_HEIGHT);// floor distance
	seg.setOptimizeCoefficients(true);
	seg.setInputCloud(modified_cloud);
	seg.segment(*inlier_indices, *coefficients);

	if (inlier_indices->indices.size() == 0)
	{
		std::cout << "Ground plane not found" << std::endl;
	}

	// Removing floor points from point cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(modified_cloud);
	extract.setIndices(inlier_indices);
	if (!bool_display_groundplane)
	{
		extract.setNegative(true);  // to remove the inliers AKA ground points, and keep the rest of the points
	}
	extract.filter(*modified_cloud);

}

void PCLClusterObject::ROIFiltering()
{
	// Using a pass through filter to only consider the points between x_min and x_max
	if (x_min != 0.0f and x_max != 0.0f)
	{
		pcl::PassThrough<pcl::PointXYZ> pass1;
		pass1.setInputCloud(modified_cloud);
		pass1.setFilterFieldName("x");
		pass1.setFilterLimits(x_min, x_max);
		pass1.filter(*modified_cloud);
	}

	// Using a pass through filter to only consider the points between y_min and y_max
	if (y_min != 0.0f and y_max != 0.0f)
	{
		pcl::PassThrough<pcl::PointXYZ> pass2;
		pass2.setInputCloud(modified_cloud);
		pass2.setFilterFieldName("y");
		pass2.setFilterLimits(y_min, y_max);
		pass2.filter(*modified_cloud);	
	}

	// Using a pass through filter to only consider the points between z_min and z_max
	if (z_min != 0.0f and z_max != 0.0f)
	{
		pcl::PassThrough<pcl::PointXYZ> pass3;
		pass3.setInputCloud(modified_cloud);
		pass3.setFilterFieldName("z");
		pass3.setFilterLimits(z_min, z_max);
		pass3.filter(*modified_cloud);
	}
}

void PCLClusterObject::EuclideanClustering()
{
	// creating a KDTree object for the search method of the extraction algo
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(modified_cloud);

	// pointcloud euclidean cluster extraction object
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; 
	// distance > CLUSTER_TOLERANCE meters then not considered in the cluster
	ec.setClusterTolerance (CLUSTER_TOLERANCE);
	// cluster MUST be greater than or equal to MIN_CLUSTER_SIZE points
	ec.setMinClusterSize (MIN_CLUSTER_SIZE);
	// cluster MUST be less than or equal to MAX_CLUSTER_SIZE points
	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
	ec.setSearchMethod (tree);
	ec.setInputCloud (modified_cloud);
	// extracted the clusters out of the point cloud and saved the indices in the cluster indices
	ec.extract (cluster_indices);
}

void PCLClusterObject::BoundingBoxes_centroids_vertices()
{
	int id = 0;
	for (auto &cluster : cluster_indices)
    {
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ box_centroid;
        for (auto &indx : cluster.indices)
        {
            // iterating through each index of a cluster and checking for the closest point, only considering y axis
            cluster_cloud->push_back(modified_cloud->points[indx]);
        }
        // finding min and max point in each axes
        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);

        // publishing bounding box marker
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.
		marker.header.frame_id = TF_FRAME_ID;
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "bounding_boxes";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.
		marker.pose.position.x = (max_point.x + min_point.x) / 2;
		marker.pose.position.y = (max_point.y + min_point.y) / 2;
		marker.pose.position.z = (max_point.z + min_point.z) / 2;

		marker.pose.orientation.x = 0.0; // rolling
		marker.pose.orientation.y = 0.0; // pitching
		marker.pose.orientation.z = 0.0; // yawing
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker 
		marker.scale.x = max_point.x - min_point.x;
		marker.scale.y = max_point.y - min_point.y;
		marker.scale.z = max_point.z - min_point.z;

		// Set the color -- green
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		// alpha value 1 is completely opaque
		marker.color.a = 0.2f;

		ros::Duration lifetime;
		marker.lifetime = lifetime.fromSec(0.155); // 1fps : 1hz and 20hz : 0.1 seconds
		markers_array.markers.push_back(marker);
		id++;

		// ---------------- CENTROIDS VERTICES ---------------- //
		if (bool_find_centroids_and_vertices)
		{
			box_centroid.x = (max_point.x + min_point.x) / 2;
			box_centroid.y = (max_point.y + min_point.y) / 2;
			box_centroid.z = (max_point.z + min_point.z) / 2;
	        // insert centroid to cluster_centroids_vector
	        centroids_cloud->push_back(box_centroid);
	        // centroids + vertices
	        std::vector<float> temp = {box_centroid.x, box_centroid.y, box_centroid.z,
	        	min_point.x, min_point.y, min_point.z,
	        	max_point.x, min_point.y, min_point.z,
	        	max_point.x, min_point.y, max_point.z,
	        	min_point.x, min_point.y, max_point.z,
	        	min_point.x, max_point.y, max_point.z,
	        	max_point.x, max_point.y, max_point.z,
	        	max_point.x, max_point.y, min_point.z,
	        	min_point.x, max_point.y, min_point.z
	        };
	        centroid_vertices.push_back(temp);
		}   
    }
}

void ROS::publish_centroids_points_and_vertices()
{
    std_msgs::Float32MultiArray dat;
    int H = pc->centroid_vertices.size(); //rows
    int W = pc->centroid_vertices[0].size(); //columns

    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "height";
    dat.layout.dim[1].label = "width";
    dat.layout.dim[0].size = H;
    dat.layout.dim[1].size = W;
    dat.layout.dim[0].stride = H*W;
    dat.layout.dim[1].stride = W;
    dat.layout.data_offset = 0;
    std::vector<float> vec(W*H, 0);
    for (int i=0; i<H; i++)
    {
        for (int j=0; j<W; j++)
            vec[i*W + j] = pc->centroid_vertices[i][j];
    }
    dat.data = vec;
    centroid_vertices_pub.publish(dat);
}


void ROS::get_ros_params(ros::NodeHandle *nh)
{
    nh->getParam("/MAX_HEIGHT", pc->MAX_HEIGHT);
    nh->getParam("/FLOOR_MAX_ANGLE", pc->FLOOR_MAX_ANGLE);
    nh->getParam("/LEAF_SIZE", pc->LEAF_SIZE);
    nh->getParam("/MAX_CLUSTER_SIZE", pc->MAX_CLUSTER_SIZE);
    nh->getParam("/MIN_CLUSTER_SIZE", pc->MIN_CLUSTER_SIZE);
    nh->getParam("/CLUSTER_TOLERANCE", pc->CLUSTER_TOLERANCE);
    nh->getParam("/bool_filter_ground", pc->bool_filter_ground);
	nh->getParam("/bool_display_groundplane", pc->bool_display_groundplane);
    nh->getParam("/bool_downsample", pc->bool_downsample);
    nh->getParam("/bool_ROI_filtering", pc->bool_ROI_filtering);
    nh->getParam("/x_min", pc->x_min); 
    nh->getParam("/x_max", pc->x_max); 
    nh->getParam("/y_min", pc->y_min); 
    nh->getParam("/y_max", pc->y_max); 
    nh->getParam("/z_min", pc->z_min); 
    nh->getParam("/z_max", pc->z_max); 
    nh->getParam("/bool_difference_normals_segmentation", pc->bool_difference_normals_segmentation);
    nh->getParam("/bool_find_centroids_and_vertices", pc->bool_find_centroids_and_vertices);
    nh->getParam("/bool_publish_bounding_box", pc->bool_publish_bounding_box);
    nh->getParam("/point_cloud_topic", pc->point_cloud_topic);
	nh->getParam("/tf_frame_id", pc->TF_FRAME_ID);
    nh->getParam("/CENTROIDS_PUB", pc->CENTROIDS_PUB);
    nh->getParam("/CENTROID_VERTICES_PUB", pc->CENTROID_VERTICES_PUB);
    nh->getParam("/PROCESSED_POINTCLOUD_PUB", pc->SEG_PUB);
    nh->getParam("/BOUNDING_BOX_PUB", pc->BOUNDING_BOX_PUB);
    nh->getParam("/bool_publish_centroids", pc->bool_publish_centroids);
    nh->getParam("/bool_publish_centroids_and_vertices", pc->bool_publish_centroids_and_vertices);
    nh->getParam("/radius_large", pc->radius_large);
    nh->getParam("/threshold", pc->threshold);
}

void PCLClusterObject::clear()
{
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    modified_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    centroids_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    centroid_vertices.clear();
    cluster_indices.clear();
    markers_array.markers.clear();
}

void PCLClusterObject::run_clustering()
{
	
	// 1. Ground Segmentation
	if (bool_filter_ground)
	{
		RemoveGroundPlane();
	}

    // 2. Downsampling
	if (bool_downsample)
	{
		Downsampling();	
	}

	// 3. ROI Filtering
	if (bool_ROI_filtering)
	{
		ROIFiltering();
	}


	// 4. Difference Normals Segmentation
	if (bool_difference_normals_segmentation)
	{
    	DifferenceOfNormalsSegmentation();
	}

    // 5.  Euclidean clustering
    EuclideanClustering();

    // 6. getting bounding boxes
    if (bool_publish_centroids_and_vertices)
    {
    	BoundingBoxes_centroids_vertices();
    }
	
}

void ROS::PCLCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// ros msg PointCloud2 --> pcl PointCloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr original_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	sensor_msgs::PointCloud2 transformed_msg;
	const sensor_msgs::PointCloud2ConstPtr& transformed_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(transformed_msg);

    pcl::fromROSMsg (*cloud_msg, *original_point_cloud);
	
	tf::TransformListener transformlistener;
	const std::string frame = pc->TF_FRAME_ID;
	pcl_ros::transformPointCloud (frame, *cloud_msg, transformed_msg, transformlistener);

	pcl::fromROSMsg (transformed_msg, *transformed_point_cloud);

    pc->set_header_name(cloud_msg->header);
	pc->set_input_cloud(transformed_point_cloud);

    pc->run_clustering();

   	// publishing centroids
   	if (pc->bool_publish_centroids and pc->bool_find_centroids_and_vertices)
   	{
   		// std::cout << "INSIDE OF THE PUBLISH CENTROIDS THING" << std::endl;
		publish_centroids();
   	}
   	// publishing centroids and vertices 
   	if (pc->bool_publish_centroids_and_vertices)
   	{
   		publish_centroids_points_and_vertices();
   	}
   	// publishing bounding boxes for visualisation
   	if (pc->bool_publish_bounding_box)
   	{
   		publish_bounding_box_marker();
   	}
	// publish_segmented_cloud();     
   	pc->clear();
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "pcl_object_detection");
    ros::NodeHandle nh;
	PCLClusterObject pc = PCLClusterObject();
	ROS r = ROS(&nh, &pc);
	ros::spin();
}