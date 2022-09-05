#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PCLClusterObject
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr modified_cloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seg_cloud;

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud;
    

    std::vector<std::vector<float>> centroid_vertices;
    visualization_msgs::MarkerArray markers_array;

    std_msgs::Header header_name;

    float MAX_HEIGHT;
    float FLOOR_MAX_ANGLE;
    float LEAF_SIZE;
    float MAX_CLUSTER_SIZE;
    int MIN_CLUSTER_SIZE;
    float CLUSTER_TOLERANCE;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min; 
    float z_max;
    float radius_small;
    float radius_large;
    float threshold;

    // booleans
    bool bool_filter_ground;
    bool bool_display_groundplane;
    bool bool_downsample;
    bool bool_ROI_filtering;
    bool bool_difference_normals_segmentation;
    bool bool_find_centroids_and_vertices;
    bool bool_publish_bounding_box;
    // params for ROS
    std::string POINTCLOUD_SUB;
    std::string CENTROIDS_PUB;
    std::string BOUNDING_BOX_PUB;
    std::string CENTROID_VERTICES_PUB;
    std::string SEG_PUB;
    std::string TF_FRAME_ID;


    bool bool_publish_centroids;
    bool bool_publish_centroids_and_vertices;

    void set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_cloud)
    {
        input_cloud = input_point_cloud;
        modified_cloud = input_point_cloud;
        // ground_seg_cloud = input_point_cloud;

    }

    void set_header_name(std_msgs::Header header)
    {
        header_name = header;
    }

protected:
    void Downsampling();
    void DifferenceOfNormalsSegmentation();
    void RemoveGroundPlane();
    void ROIFiltering();
    void EuclideanClustering();
    void BoundingBoxes_centroids_vertices();
    void PCLCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

public:
    void clear();
    void run_clustering();

    PCLClusterObject()
    {
        input_cloud.reset( new pcl::PointCloud<pcl::PointXYZ>);
        modified_cloud.reset( new pcl::PointCloud<pcl::PointXYZ>);
        // ground_seg_cloud.reset( new pcl::PointCloud<pcl::PointXYZ>);
        centroids_cloud.reset( new pcl::PointCloud<pcl::PointXYZ>);
        cluster_indices.clear();
        centroid_vertices.clear();
        markers_array.markers.clear();
    }
};

class ROS
{
protected:
    // ROS publishers
    ros::Publisher pcl_centroids_pub;
    ros::Publisher pcl_bounding_boxes_pub;
    ros::Publisher centroid_vertices_pub;
    // ros::Publisher segmented_cloud_pub;

    //ROS subscribers
    ros::Subscriber sub_pcl;
    
    std::unique_ptr<PCLClusterObject> pc;

    void get_ros_params(ros::NodeHandle *nh);
    void publish_centroids()
    {
        sensor_msgs::PointCloud2 centroids_cloud_publish;
        pcl::toROSMsg(*(pc->centroids_cloud), centroids_cloud_publish);
        centroids_cloud_publish.header = pc->header_name;
        pcl_centroids_pub.publish(centroids_cloud_publish);
    }

    void publish_centroids_points_and_vertices();
    void publish_bounding_box_marker()
    {
        pcl_bounding_boxes_pub.publish(pc->markers_array);
    }

    // void publish_segmented_cloud()
    // {
    //     sensor_msgs::PointCloud2 pointcloud_processed;
    //     pcl::toROSMsg(*(pc->ground_seg_cloud), pointcloud_processed);
    //     pointcloud_processed.header = pc->header_name;
    //     segmented_cloud_pub.publish(pointcloud_processed);
    // }

    void PCLCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

public:
    ROS(ros::NodeHandle *nh, PCLClusterObject *pc_ptr)
    {
        tf::TransformListener transformlistener;
        pc.reset(new PCLClusterObject());
        get_ros_params(nh);
        sub_pcl = nh->subscribe(pc->POINTCLOUD_SUB, 1, &ROS::PCLCallBack, this);

        pcl_centroids_pub = nh->advertise<sensor_msgs::PointCloud2> (pc->CENTROIDS_PUB, 30);  
        pcl_bounding_boxes_pub = nh->advertise<visualization_msgs::MarkerArray> (pc->BOUNDING_BOX_PUB, 30);
        centroid_vertices_pub = nh->advertise<std_msgs::Float32MultiArray>(pc->CENTROID_VERTICES_PUB, 30);
        // segmented_cloud_pub = nh->advertise<sensor_msgs::PointCloud2> (pc->SEG_PUB, 30);

    }
};