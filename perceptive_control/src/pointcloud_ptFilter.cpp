/*
 * @Author: haoyun 
 * @Date: 2023-06-17 15:18:36
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-19 10:13:02
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/pointcloud_ptFilter.cpp
 * @Description: crop the points that close to the camera using PCL/PassThrough filter
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"

class crop_outlier
{
 public:
    crop_outlier(ros::NodeHandle& nh): _nh(nh) {
        src_point_cloud_subscriber = _nh.subscribe("camera/depth/color/points", 10,
                                                    &crop_outlier::cloud_update_callback, this);
        filtered_point_cloud_publisher = _nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);                                       
 
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(0.9, 6); // in meter

        // passThroughFilter.setFilterFieldName("y");
        // passThroughFilter.setFilterLimits(-100.0, 0); // in meter

        outlierFilter.setRadiusSearch(0.05);
        outlierFilter.setMinNeighborsInRadius(10);

        
    }
    ~crop_outlier() {
        
    }
    void cloud_update_callback(const sensor_msgs::PointCloud2ConstPtr input) {
        
        point_cloud_pub_data.header = input.get()->header;

        pcl::fromROSMsg(*input, *cloud_in);
        passThroughFilter.setInputCloud(cloud_in);
        passThroughFilter.filter(*cloud_filtered_1);

        outlierFilter.setInputCloud(cloud_filtered_1);
        outlierFilter.filter(*cloud_filtered_2);

        pcl::toROSMsg(*cloud_filtered_2, point_cloud_pub_data);
        
        filtered_point_cloud_publisher.publish(point_cloud_pub_data);
    }

    ros::NodeHandle _nh;
    ros::Subscriber src_point_cloud_subscriber;
    ros::Publisher filtered_point_cloud_publisher;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 {new pcl::PointCloud<pcl::PointXYZ>};


    sensor_msgs::PointCloud2 point_cloud_pub_data;
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlierFilter;


};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointcloud_filters_init_name_2");
    ros::NodeHandle nh;
    ROS_INFO("start pointcloud_outliers_filters node ...");
    crop_outlier pointsFilter(nh);
    ros::spin();
    return 0;
}

