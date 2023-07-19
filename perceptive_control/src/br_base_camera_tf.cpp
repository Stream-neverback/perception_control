/*
 * @Author: haoyun 
 * @Date: 2023-05-09 14:58:06
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-11 19:13:19
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/br_base_camera_tf.cpp
 * @Description: broadcast the tf2 between the robot's base and camera
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "perceptive_control/robotStates.h"
#include "perceptionlcm/robot_states.hpp"
#include "lcm/lcm-cpp.hpp"

class brBaseCameraTf
{
 public:
    brBaseCameraTf(ros::NodeHandle &nh):_nh(nh) { }
    ~brBaseCameraTf() { }
    void init() {
        mapCameraTransformStamped.header.frame_id = "map";
        mapCameraTransformStamped.child_frame_id = "camera_link";
        
        mapHRItransformStamped.header.frame_id = "map";
        mapHRItransformStamped.child_frame_id = "hri";
        // this->r_human_waist = _nh.param("diameter_human_waist", 0.20);
        // this->delta_h = _nh.param("delta_height", -0.1);
        this->r_human_waist = 0.22;
        this->delta_h = -0.19;
        ros::param::get("~diameter_human_waist", this->r_human_waist);
        ros::param::get("~delta_height", this->delta_h);
        // std::cout << "dekta h = " << this->delta_h << std::endl;
        // ROS_INFO("ss");

        check_top_tfStamped.header.frame_id = "map";
        check_top_tfStamped.child_frame_id = "check_top_frame";

        check_left_tfStamped.header.frame_id = "map";
        check_left_tfStamped.child_frame_id = "check_left_frame";

        check_right_tfStamped.header.frame_id = "map";
        check_right_tfStamped.child_frame_id = "check_right_frame";
        
        
    }
    void subscribeRobotState() {
        _sub = _nh.subscribe<perceptive_control::robotStates>("robot_states", 5,
         &brBaseCameraTf::callback, this);
    }

    void callback(const perceptive_control::robotStates::ConstPtr& state_ptr) {
        ros::Time stamp = ros::Time::now();
        // std::cout << "" << std::endl;
        // ROS_INFO("publish map camera tf");
        // Step #1: calculate the human-robot interaction position
        Eigen::Vector3f world_p_hri; // the position of HRI, expressed in the world frame
        Eigen::Vector3f body_r_body_human;  // the vector from body to HRI, expressed in the world frame
        // body_r_body_human << 0.4846, 0.0, 0.15;
        body_r_body_human << 0.48, 0.0, 0.15;

        Eigen::Vector3f base_pos;
        base_pos << state_ptr->position[0], state_ptr->position[1], state_ptr->position[2];
        Eigen::Matrix3f world_R_body;
        Eigen::Quaternionf quat_base(state_ptr->orientation_wxyz[0],state_ptr->orientation_wxyz[1],
                                    state_ptr->orientation_wxyz[2],state_ptr->orientation_wxyz[3]);
        world_R_body = quat_base.toRotationMatrix();
        

        // debug: transpose?
        world_p_hri = base_pos + world_R_body*body_r_body_human;

        // Step #2: waist center
        Eigen::Vector3f world_p_waistCenter, radius_height;
        Eigen::Vector3f rpy = quatToRPY(quat_base);
        Eigen::Matrix3f Ryaw;
        Ryaw << cos(rpy(2)), -sin(rpy(2)), 0,
                sin(rpy(2)), cos(rpy(2)), 0,
                0, 0, 1;
        radius_height << r_human_waist/2, 0, delta_h;
        //debug: transpose(?)
        world_p_waistCenter = world_p_hri + Ryaw*radius_height;

        // Step #3: camera pos
        Eigen::Vector3f world_p_camera;
        Eigen::Matrix3f world_R_camera;
        
        Eigen::Quaternionf quat_camera(state_ptr->camera_quat[0],state_ptr->camera_quat[1],
                                    state_ptr->camera_quat[2],state_ptr->camera_quat[3]);
        world_R_camera = quat_camera.toRotationMatrix();
        Eigen::Vector3f rpy_camera = quatToRPY(quat_camera);
        Eigen::Matrix3f Ryaw_camera;
        Ryaw_camera << cos(rpy_camera(2)), -sin(rpy_camera(2)), 0,
                       sin(rpy_camera(2)), cos(rpy_camera(2)), 0,
                       0, 0, 1;
        
        Eigen::Vector3f radius; radius << r_human_waist/2, 0, 0;
        // debug: transpose?
        world_p_camera = world_p_waistCenter + Ryaw_camera*radius;
        

        // Step #4: send
        mapHRItransformStamped.header.stamp = stamp;
        mapHRItransformStamped.transform.translation.x = world_p_hri[0];
        mapHRItransformStamped.transform.translation.y = world_p_hri[1];
        mapHRItransformStamped.transform.translation.z = world_p_hri[2];
        Eigen::Quaternionf Rot_hri_yaw(Ryaw); // construct a quaternion from a rotation matrix expression
        mapHRItransformStamped.transform.rotation.w = Rot_hri_yaw.w();
        mapHRItransformStamped.transform.rotation.x = Rot_hri_yaw.x();
        mapHRItransformStamped.transform.rotation.y = Rot_hri_yaw.y();
        mapHRItransformStamped.transform.rotation.z = Rot_hri_yaw.z();
        br.sendTransform(mapHRItransformStamped);



        mapCameraTransformStamped.header.stamp = stamp;
        mapCameraTransformStamped.transform.translation.x = world_p_camera[0];
        mapCameraTransformStamped.transform.translation.y = world_p_camera[1];
        mapCameraTransformStamped.transform.translation.z = world_p_camera[2];

        mapCameraTransformStamped.transform.rotation.w = state_ptr->camera_quat[0];
        mapCameraTransformStamped.transform.rotation.x = state_ptr->camera_quat[1];
        mapCameraTransformStamped.transform.rotation.y = state_ptr->camera_quat[2];
        mapCameraTransformStamped.transform.rotation.z = state_ptr->camera_quat[3];

        br.sendTransform(mapCameraTransformStamped);

        // broadcast the checkpoint tf. comment the below codes if we are not calibrating
        check_left_tfStamped.header.stamp = stamp;
        // check_left_tfStamped.transform.translation.x = world_p_camera[0] + 0.3;
        check_left_tfStamped.transform.translation.x = base_pos[0] + 1.483;
        check_left_tfStamped.transform.translation.y = base_pos[1] + 0.17;
        check_left_tfStamped.transform.translation.z = 0.0;
        check_left_tfStamped.transform.rotation.w = 1.0;
        check_left_tfStamped.transform.rotation.x = 0.0;
        check_left_tfStamped.transform.rotation.y = 0.0;
        check_left_tfStamped.transform.rotation.z = 0.0;
        br.sendTransform(check_left_tfStamped);

        check_right_tfStamped.header.stamp = stamp;
        // check_right_tfStamped.transform.translation.x = world_p_camera[0] + 0.3;
        check_right_tfStamped.transform.translation.x = base_pos[0] + 1.483;
        check_right_tfStamped.transform.translation.y = base_pos[1] - 0.17;
        check_right_tfStamped.transform.translation.z = 0.0;
        check_right_tfStamped.transform.rotation.w = 1.0;
        check_right_tfStamped.transform.rotation.x = 0.0;
        check_right_tfStamped.transform.rotation.y = 0.0;
        check_right_tfStamped.transform.rotation.z = 0.0;
        br.sendTransform(check_right_tfStamped);

        check_top_tfStamped.header.stamp = stamp;
        // check_top_tfStamped.transform.translation.x = world_p_camera[0] + 0.5;
        check_top_tfStamped.transform.translation.x = base_pos[0] + 1.283;
        check_top_tfStamped.transform.translation.y = base_pos[1];
        check_top_tfStamped.transform.translation.z = 0.0;
        check_top_tfStamped.transform.rotation.w = 1.0;
        check_top_tfStamped.transform.rotation.x = 0.0;
        check_top_tfStamped.transform.rotation.y = 0.0;
        check_top_tfStamped.transform.rotation.z = 0.0;
        br.sendTransform(check_top_tfStamped);
        

    }

    Eigen::Vector3f quatToRPY(const Eigen::Quaternionf& quat) {
        Eigen::Vector3f rpy;
        // MIT-Cheetah order: q = [w, x, y, z]
        Eigen::Vector4f q;
        q << quat.w(), quat.x(), quat.y(), quat.z();
        float as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
        rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                 q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
        rpy(1) = std::asin(as);
        rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                 q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
        
        return rpy;
    }

    // Eigen::Vector4f rotationMatrixToQuaternion(const Eigen::Matrix3f r) {
    //     Eigen::Vector4f q;

    // }

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    geometry_msgs::TransformStamped mapCameraTransformStamped;
    geometry_msgs::TransformStamped mapHRItransformStamped;
    geometry_msgs::TransformStamped check_top_tfStamped, check_left_tfStamped, check_right_tfStamped;

    tf2_ros::TransformBroadcaster br;

    // params:
    float r_human_waist;
    float delta_h;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "broadcast_base_camera_tf_node");
    ros::NodeHandle nh;
    ROS_INFO("start broadcast_base_camera_tf node ...");
    brBaseCameraTf br_base_camera_tf(nh);
    br_base_camera_tf.init();
    br_base_camera_tf.subscribeRobotState();

    ros::spin();
    return 0;

}