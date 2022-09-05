#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

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

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class GroundSegmentation
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_segmented_cloud;

    std_msgs::Header header_name;

    // booleans
    bool bool_filter_ground;
    bool bool_display_groundplane;
    float MAX_HEIGHT;
    float FLOOR_MAX_ANGLE;
    // params for ROS
    std::string LIDAR_TOPIC_SUBSCRIBE;
    std::string SEG_PUB;



    void set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_cloud)
    {
        input_cloud = input_point_cloud;
        ground_segmented_cloud = input_point_cloud;
    }

    void set_header_name(std_msgs::Header header)
    {
        header_name = header;
    }

protected:
    void RemoveGroundPlane();
    void PCLCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

public:
    void clear();
    void run_groundplane_seg();

    GroundSegmentation()
    {
        input_cloud.reset( new pcl::PointCloud<pcl::PointXYZ>);
        ground_segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
};

class ROS
{
protected:
    // ROS publishers
    ros::Publisher segmented_cloud_pub;

    //ROS subscribers
    ros::Subscriber sub_lidar;
    ros::Subscriber sub_lanes;

    std::unique_ptr<GroundSegmentation> gs_obj;

    void get_ros_params(ros::NodeHandle *nh);
    void publish_segmented_cloud()
    {
        sensor_msgs::PointCloud2 ground_filtered_cloud;
        pcl::toROSMsg(*(gs_obj->ground_segmented_cloud), ground_filtered_cloud);
        ground_filtered_cloud.header = gs_obj->header_name;
        segmented_cloud_pub.publish(ground_filtered_cloud);
    }


    void PCLCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

public:
    ROS(ros::NodeHandle *nh, GroundSegmentation *lc_ptr)
    {
        gs_obj.reset(new GroundSegmentation());
        get_ros_params(nh);
        sub_lidar = nh->subscribe(gs_obj->LIDAR_TOPIC_SUBSCRIBE, 1, &ROS::PCLCallBack, this);
        segmented_cloud_pub = nh->advertise<sensor_msgs::PointCloud2> (gs_obj->SEG_PUB, 30);

    }
};