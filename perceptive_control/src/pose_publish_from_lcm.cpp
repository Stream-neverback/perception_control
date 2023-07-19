/*
 * @Author: haoyun 
 * @Date: 2022-12-08 10:44:54
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-14 16:09:10
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/pose_publish_from_lcm.cpp
 * @Description: RePublish the pose/tf rostopic from lcm
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include "perceptive_control/robotStates.h"

#include "exlcm/example_t.hpp"
#include "perceptionlcm/robot_states.hpp"
#include <lcm/lcm-cpp.hpp>

namespace lcm_process {

class PoseHandler {
  public:
    PoseHandler(ros::NodeHandle& nh) {
        this->_nh = nh;
    }
    ~PoseHandler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const exlcm::example_t *msg)
    {
        int i;
        printf("Received message on channel \"%s\":\n", chan.c_str());
        printf("  timestamp   = %lld\n", (long long) msg->timestamp);
        printf("  position    = (%f, %f, %f)\n", msg->position[0], msg->position[1],
               msg->position[2]);
        printf("  orientation = (%f, %f, %f, %f)\n", msg->orientation[0], msg->orientation[1],
               msg->orientation[2], msg->orientation[3]);
        printf("  ranges:");
        for (i = 0; i < msg->num_ranges; i++)
            printf(" %d", msg->ranges[i]);
        printf("\n");
        printf("  name        = '%s'\n", msg->name.c_str());
        printf("  enabled     = %d\n", msg->enabled);
    }

    ros::NodeHandle _nh;

};


// double last_time = 0;
class RobotStateCallback {
 public:
    RobotStateCallback(ros::NodeHandle& nh) { 
        this->_nh = nh; 
        this->_robot_states_pub = this->_nh.advertise<perceptive_control::robotStates>("robot_states", 100);
    }
    ~RobotStateCallback() {;}

    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const perceptionlcm::robot_states *msg)
    {
        // printf("Received message on channel \"%s\":\n", chan.c_str());
        // double now = ros::Time::now().toSec();
        // std::cout << now - last_time << std::endl;
        // last_time = now;
        _states.control_mode = msg->control_mode;
        for (int i = 0; i < 3; i++) {
            _states.position[i] = msg->position[i];
            _states.orientation_wxyz[i] = msg->orientation_wxyz[i];
            _states.nominal_footholds[i] = msg->nominal_footholds[i];
            _states.swing_foot_init_pos[i] = msg->swing_foot_init_pos[i];


            _states.left_foot_pos[i] = msg->left_foot_pos[i];
            _states.left_foot_quat[i] = msg->left_foot_quat[i];

            _states.right_foot_pos[i] = msg->right_foot_pos[i];
            _states.right_foot_quat[i] = msg->right_foot_quat[i];
            _states.camera_quat[i] = msg->camera_quat[i];

        }
        _states.orientation_wxyz[3] = msg->orientation_wxyz[3];
        _states.left_foot_quat[3] = msg->left_foot_quat[3];
        _states.right_foot_quat[3] = msg->right_foot_quat[3];
        _states.camera_quat[3] = msg->camera_quat[3];



        for (int i = 3; i < 6; i++) {
            _states.nominal_footholds[i] = msg->nominal_footholds[i];
            _states.swing_foot_init_pos[i] = msg->swing_foot_init_pos[i];
        }
        
        for (int leg = 0; leg < 2; leg++)
        {
            _states.plan_stances[leg] = msg->plan_stances[leg];
            _states.plan_swings[leg] = msg->plan_swings[leg];
            _states.swing_total_time[leg] = msg->default_swing_total_time[leg];
        }
         


        _robot_states_pub.publish(_states);

    }


    ros::NodeHandle _nh;
    ros::Publisher _robot_states_pub;
    perceptive_control::robotStates _states;
};
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "init_name_pose_publisher");
    ros::NodeHandle nh;

    // lcm::LCM lcm_sub_pose("udpm://239.255.76.67:7667?ttl=1");
    // if(lcm_sub_pose.good())
    //     ROS_INFO("LCM init okay!");

    // lcm_process::PoseHandler handlerObject(nh);
    // lcm_sub_pose.subscribe("EXAMPLE", &lcm_process::PoseHandler::handleMessage, &handlerObject);

    // while (ros::ok()) {
    //     lcm_sub_pose.handle();
    // }

    // tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.frame_id = "map";
    // transformStamped.child_frame_id = "base";

    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.6;

    // tf2::Quaternion quat;
    // quat.setRPY(0, 30.0*M_PI/180.0, 0);
    // transformStamped.transform.rotation.x = quat.x();
    // transformStamped.transform.rotation.y = quat.y();
    // transformStamped.transform.rotation.z = quat.z();
    // transformStamped.transform.rotation.w = quat.w();

    // geometry_msgs::TransformStamped transformStamped_base_camera;
    // transformStamped_base_camera.header.frame_id = "base";
    // transformStamped_base_camera.child_frame_id = "camera_link";
    // transformStamped_base_camera.transform.translation.x = 0.0;
    // transformStamped_base_camera.transform.translation.y = 0.0;
    // transformStamped_base_camera.transform.translation.z = 0.0;

    // tf2::Quaternion quat2;
    // quat2.setRPY(0, 0, 0);
    // transformStamped_base_camera.transform.rotation.x = quat2.x();
    // transformStamped_base_camera.transform.rotation.y = quat2.y();
    // transformStamped_base_camera.transform.rotation.z = quat2.z();
    // transformStamped_base_camera.transform.rotation.w = quat2.w();
    
    
    // ros::Rate rate(2);
    // while (ros::ok()) {

    //     transformStamped.header.stamp = ros::Time::now();
    //     transformStamped_base_camera.header.stamp = ros::Time::now();
    //     // br.sendTransform(transformStamped);
    //     // br.sendTransform(transformStamped_base_camera);
    //     // ROS_INFO("broadcast tf ...");
    //     rate.sleep();
    // }


    lcm::LCM lcm_sub_pose("udpm://239.255.76.67:7667?ttl=1");
    if(lcm_sub_pose.good())
        ROS_INFO("LCM init okay!");

    lcm_process::RobotStateCallback handlerObject(nh);
    lcm_sub_pose.subscribe("ROBOT_STATES", &lcm_process::RobotStateCallback::handleMessage, &handlerObject);

    while (ros::ok()) {
        lcm_sub_pose.handle();
    }
    


    ros::spin();
    
    return 0;
}
