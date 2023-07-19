/*
 * @Author: haoyun 
 * @Date: 2023-01-02 21:08:48
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-02-28 15:04:58
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/pointcloud_filters.cpp
 * @Description: discard the points at the location of robotic-feet
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include "perceptive_control/robotStates.h"

class crop_leg_points
{
 public:
    crop_leg_points(ros::NodeHandle& nh):_nh(nh) {
        sub_src_point_cloud_ = _nh.subscribe("camera/depth/color/points", 10,
                                             &crop_leg_points::filter_callback, this);

        pub_filted_point_cloud_ = _nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);


        min_point << -0.20, -0.20, -0.2, 1.0;
        max_point << 0.20, 0.20 , 0.48, 1.0;
        // min_point << -0.15, -0.15, -0.15, 1.0;
        // max_point << 0.15, 0.15 , 0.48, 1.0;

        for (int i = 0; i < 2; i++) {
            r_box_center[i].setZero();
            rpy_box_center[i].setZero();
        }
        
        start_filter = false;


        cropBoxFilter[0].setNegative(true);
        cropBoxFilter[1].setNegative(true);

        cropBoxFilter[0].setMin(min_point);
        cropBoxFilter[0].setMax(max_point);

        cropBoxFilter[1].setMin(min_point);
        cropBoxFilter[1].setMax(max_point);

    }
    ~crop_leg_points() {}
    void spin() 
    {

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(400);
        while (ros::ok())
        {
            try
            {
                left_box_center_transform_stamped = tfBuffer.lookupTransform("camera_depth_optical_frame", "left_foot_frame",
                                ros::Time(0));
                // ROS_INFO("fatherID = %s ", left_box_center_transform_stamped.header.frame_id.c_str());

                r_box_center[0](0) = left_box_center_transform_stamped.transform.translation.x;
                r_box_center[0](1) = left_box_center_transform_stamped.transform.translation.y;
                r_box_center[0](2) = left_box_center_transform_stamped.transform.translation.z;

                quat_box[0].setW(left_box_center_transform_stamped.transform.rotation.w);
                quat_box[0].setX(left_box_center_transform_stamped.transform.rotation.x);
                quat_box[0].setY(left_box_center_transform_stamped.transform.rotation.y);
                quat_box[0].setZ(left_box_center_transform_stamped.transform.rotation.z);

                double roll, pitch, yaw;
                tf2::Matrix3x3(quat_box[0]).getRPY(roll, pitch, yaw);
                rpy_box_center[0](0) = roll;
                rpy_box_center[0](1) = pitch;
                rpy_box_center[0](2) = yaw;
                
            }
            catch(tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
            }

            try
            {
                right_box_center_transform_stamped = tfBuffer.lookupTransform("camera_depth_optical_frame", "right_foot_frame",
                                ros::Time(0));
                r_box_center[1](0) = right_box_center_transform_stamped.transform.translation.x;
                r_box_center[1](1) = right_box_center_transform_stamped.transform.translation.y;
                r_box_center[1](2) = right_box_center_transform_stamped.transform.translation.z;

                quat_box[1].setW(right_box_center_transform_stamped.transform.rotation.w);
                quat_box[1].setX(right_box_center_transform_stamped.transform.rotation.x);
                quat_box[1].setY(right_box_center_transform_stamped.transform.rotation.y);
                quat_box[1].setZ(right_box_center_transform_stamped.transform.rotation.z);

                double roll, pitch, yaw;
                tf2::Matrix3x3(quat_box[1]).getRPY(roll, pitch, yaw);
                rpy_box_center[1](0) = roll;
                rpy_box_center[1](1) = pitch;
                rpy_box_center[1](2) = yaw;

                start_filter = true;
            }
            catch(tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }

            rate.sleep();
            ros::spinOnce();
            
        }
        
        

    }
    void filter_callback(const sensor_msgs::PointCloud2ConstPtr input)
    {
        // std::cout << "receive input" << std::endl;
        stamp_ = input.get()->header.stamp;
        pcl::fromROSMsg(*input, *cloud_in);

        if (!start_filter)
        {
            pcl::toROSMsg(*cloud_in, cloud_pub);
        }
        else {
            // left foot filter
            // ROS_INFO("start filters ..");
            cropBoxFilter[0].setTranslation(r_box_center[0]);
            cropBoxFilter[0].setRotation(rpy_box_center[0]);

            cropBoxFilter[0].setInputCloud(cloud_in);
            cropBoxFilter[0].filter(*cloud_filted_1);

             // right foot filter
            cropBoxFilter[1].setTranslation(r_box_center[1]);
            cropBoxFilter[1].setRotation(rpy_box_center[1]);

            cropBoxFilter[1].setInputCloud(cloud_filted_1);
            cropBoxFilter[1].filter(*cloud_filted_2);

            pcl::toROSMsg(*cloud_filted_2, cloud_pub);

            publish_box_coordinate();
        }
        
        cloud_pub.header = input.get()->header;
        pub_filted_point_cloud_.publish(cloud_pub);
        

    }
    void publish_box_coordinate() 
    {
        // left_box_center_transform_stamped.transform.translation.x = r_box_center[0](0);
        // left_box_center_transform_stamped.transform.translation.y = r_box_center[1](1);
        // left_box_center_transform_stamped.transform.translation.z = r_box_center[2];

        // tf2::Quaternion quat_left_box;
       
        // quat_left_box.setRPY(rpy_box_center[0], rpy_box_center[1], rpy_box_center[2]);
        // left_box_center_transform_stamped.transform.rotation.w = quat_left_box.w();
        // left_box_center_transform_stamped.transform.rotation.x = quat_left_box.x();
        // left_box_center_transform_stamped.transform.rotation.y = quat_left_box.y();
        // left_box_center_transform_stamped.transform.rotation.z = quat_left_box.z();

        // left_box_center_transform_stamped.header.stamp = stamp_;
        // br.sendTransform(left_box_center_transform_stamped);
    }

    ros::NodeHandle _nh;
    ros::Subscriber sub_src_point_cloud_, sub_rob_states_;
    ros::Publisher pub_filted_point_cloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filted_1 {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filted_2 {new pcl::PointCloud<pcl::PointXYZ>};

    sensor_msgs::PointCloud2 cloud_pub;

    bool start_filter;
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter[2];
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    Eigen::Vector3f r_box_center[2];
    tf2::Quaternion quat_box[2];
    Eigen::Vector3f rpy_box_center[2];

    ros::Time stamp_;
    geometry_msgs::TransformStamped left_box_center_transform_stamped;
    geometry_msgs::TransformStamped right_box_center_transform_stamped;
    tf2_ros::TransformBroadcaster br;

    


};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointcloud_filters_init_name");
    ros::NodeHandle nh;
    ROS_INFO("start pointcloud_filters node ...");
    crop_leg_points pointsFilter(nh);
    pointsFilter.spin();
    return 0;
}
