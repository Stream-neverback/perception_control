/*
 * @Author: haoyun 
 * @Date: 2022-12-22 20:31:24
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-14 16:50:51
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/br_states_to_tf.cpp
 * @Description: subscribe the robotStates topic and broadcast the corresponding tf2
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "perceptive_control/robotStates.h"
#include "perceptionlcm/robot_states.hpp"
#include <lcm/lcm-cpp.hpp>

class brStatesTf
{
 public:
    brStatesTf(ros::NodeHandle &nh):_nh(nh) { }
    ~brStatesTf() { }
    void initTfStamp() {
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base";
        
        left_foot_tf_stamp.header.frame_id = "map";
        left_foot_tf_stamp.child_frame_id = "left_foot_frame";
        
        right_foot_tf_stamp.header.frame_id = "map";
        right_foot_tf_stamp.child_frame_id = "right_foot_frame";

    }
    void subs() {
        _sub = _nh.subscribe<perceptive_control::robotStates>("robot_states", 5, &brStatesTf::callback, this);
        
    }
    void callback(const perceptive_control::robotStates::ConstPtr& states_ptr) {
        ros::Time stamp = ros::Time::now();

        transformStamped.transform.translation.x = states_ptr->position[0];
        transformStamped.transform.translation.y = states_ptr->position[1];
        transformStamped.transform.translation.z = states_ptr->position[2];

        transformStamped.transform.rotation.w = states_ptr->orientation_wxyz[0];
        transformStamped.transform.rotation.x = states_ptr->orientation_wxyz[1];
        transformStamped.transform.rotation.y = states_ptr->orientation_wxyz[2];
        transformStamped.transform.rotation.z = states_ptr->orientation_wxyz[3];
        transformStamped.header.stamp = stamp;


        left_foot_tf_stamp.transform.translation.x = states_ptr->left_foot_pos[0];
        left_foot_tf_stamp.transform.translation.y = states_ptr->left_foot_pos[1];
        left_foot_tf_stamp.transform.translation.z = states_ptr->left_foot_pos[2];

        left_foot_tf_stamp.transform.rotation.w = states_ptr->left_foot_quat[0];
        left_foot_tf_stamp.transform.rotation.x = states_ptr->left_foot_quat[1];
        left_foot_tf_stamp.transform.rotation.y = states_ptr->left_foot_quat[2];
        left_foot_tf_stamp.transform.rotation.z = states_ptr->left_foot_quat[3];
        left_foot_tf_stamp.header.stamp = stamp;


        right_foot_tf_stamp.transform.translation.x = states_ptr->right_foot_pos[0];
        right_foot_tf_stamp.transform.translation.y = states_ptr->right_foot_pos[1];
        right_foot_tf_stamp.transform.translation.z = states_ptr->right_foot_pos[2];

        right_foot_tf_stamp.transform.rotation.w = states_ptr->right_foot_quat[0];
        right_foot_tf_stamp.transform.rotation.x = states_ptr->right_foot_quat[1];
        right_foot_tf_stamp.transform.rotation.y = states_ptr->right_foot_quat[2];
        right_foot_tf_stamp.transform.rotation.z = states_ptr->right_foot_quat[3];
        right_foot_tf_stamp.header.stamp = stamp;

        br.sendTransform(transformStamped);
        br.sendTransform(left_foot_tf_stamp);
        br.sendTransform(right_foot_tf_stamp);

    }
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped left_foot_tf_stamp;
    geometry_msgs::TransformStamped right_foot_tf_stamp;
    tf2_ros::TransformBroadcaster br;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "br_states_to_tf_init_name");
    ros::NodeHandle nh;
    ROS_INFO("start br_states_to_tf node ...");

    brStatesTf br_states_tf(nh);

    br_states_tf.initTfStamp();
    br_states_tf.subs();

    ros::spin();
    return 0;
}
