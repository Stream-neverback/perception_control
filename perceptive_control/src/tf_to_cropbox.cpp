/*
 * @Author: haoyun 
 * @Date: 2023-01-04 16:00:30
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-01-04 16:05:03
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/tf_to_cropbox.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(400);
    while (ros::ok())
    {
        try
        {
        //   解析 son1 中的点相对于 son2 的坐标
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("camera_depth_optical_frame","left_foot_frame",ros::Time(0));
            ROS_INFO("Father ID=%s",tfs.header.frame_id.c_str());
            ROS_INFO("Son ID=%s",tfs.child_frame_id.c_str());
            ROS_INFO("left_foot_frame relative camera_depth_optical_frame :x=%.2f,y=%.2f,z=%.2f",
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z
                    );

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("%s",e.what());
        }
        r.sleep();

        ros::spinOnce();
    }


    return 0;
}
