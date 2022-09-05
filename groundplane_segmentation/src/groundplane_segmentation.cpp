#include "groundplane_segmentation.h"

void GroundSegmentation::RemoveGroundPlane()
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
	seg.setInputCloud(ground_segmented_cloud);
	seg.segment(*inlier_indices, *coefficients);

	if (inlier_indices->indices.size() == 0)
	{
		std::cout << "Ground plane not found" << std::endl;
	}
	else{
		std::cout<< "Ground Plane found" << std::endl;
	}

	// Removing floor points from point cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(ground_segmented_cloud);
	extract.setIndices(inlier_indices);
	if (!bool_display_groundplane)
	{
		extract.setNegative(true);  // to remove the inliers AKA ground points, and keep the rest of the points
	}
	extract.filter(*ground_segmented_cloud);
}

void ROS::get_ros_params(ros::NodeHandle *nh)
{
    nh->getParam("/MAX_HEIGHT", gs_obj->MAX_HEIGHT);
    nh->getParam("/FLOOR_MAX_ANGLE", gs_obj->FLOOR_MAX_ANGLE);
    nh->getParam("/LIDAR_TOPIC_SUBSCRIBE", gs_obj->LIDAR_TOPIC_SUBSCRIBE);
    nh->getParam("/bool_filter_ground", gs_obj->bool_filter_ground);
	nh->getParam("/bool_display_groundplane", gs_obj->bool_display_groundplane);
    nh->getParam("/SEG_PUB", gs_obj->SEG_PUB);
}


void GroundSegmentation::clear()
{
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void GroundSegmentation::run_groundplane_seg()
{
	if (bool_filter_ground)
	{
		RemoveGroundPlane();
	}
}

void ROS::PCLCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// ros msg PointCloud2 --> pcl PointCloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr original_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *original_point_cloud);

    gs_obj->set_header_name(cloud_msg->header);
	gs_obj->set_input_cloud(original_point_cloud);

    gs_obj->run_groundplane_seg();

   	// publishing bounding boxes for visualisation
   	if (gs_obj->bool_filter_ground)
   	{
        publish_segmented_cloud();
   	}

   	gs_obj->clear();
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "ground_plane_segmentation");
    ros::NodeHandle nh;
	GroundSegmentation lc = GroundSegmentation();
	ROS r = ROS(&nh, &lc);
	ros::spin();
}