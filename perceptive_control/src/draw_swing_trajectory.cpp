/*
 * @Author: haoyun 
 * @Date: 2023-06-12 14:42:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-19 19:35:02
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/draw_swing_trajectory.cpp
 * @Description: draw the swing trajectory in rviz using visualization_msgs/Marker
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_core/grid_map_core.hpp>

#include "perceptive_control/robotStates.h"
#include "perceptive_control/perceptiveAdapationInfo.h"

#include "trajectory_planning/FootSwingTrajectory.h"

class draw_lines
{
    public:
        draw_lines(ros::NodeHandle &nh):_nh(nh) {
            this->_original_line_pub[0] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("original_left_trajectory", 10);
            this->_original_line_pub[1] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("original_right_trajectory", 10);

            this->_adaption_line_pub[0] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("adption_left_trajectory", 10);
            this->_adaption_line_pub[1] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("adption_right_trajectory", 10);

            this->_history_line_pub[0] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("history_left_trajectory", 10);

            this->_history_line_pub[1] = this->_nh.advertise<visualization_msgs::Marker>
                                        ("history_right_trajectory", 10);                            

            this->_states_sub = _nh.subscribe<perceptive_control::robotStates>
                                        ("robot_states", 5, &draw_lines::callback, this);

            this->_adaption_sub = _nh.subscribe<perceptive_control::perceptiveAdapationInfo>
                                        ("adaption_info", 5, &draw_lines::adaption_callback, this);


            this->running_hz = 100;

            this->last_update_time2 = this->last_update_time1 = ros::Time::now().toSec();
            

            this->default_swing_height = 0.08;
            ros::param::get("~default_swing_height", this->default_swing_height);

            this->resolution = 10;
            ros::param::get("~resolution", this->resolution);

            this->step = 0;

            for (int foot = 0; foot < 2; foot++)
            {
                original_line_strip[foot].header.frame_id = "map";
                original_line_strip[foot].ns = "lines_ns";
                original_line_strip[foot].action = visualization_msgs::Marker::ADD;
                original_line_strip[foot].pose.orientation.w = 1.0;
                original_line_strip[foot].id = foot;
                original_line_strip[foot].type = visualization_msgs::Marker::LINE_STRIP;
                original_line_strip[foot].scale.x = 0.01;
                original_line_strip[foot].color.r = 21.0/255;
                original_line_strip[foot].color.g = 168.0/255;
                original_line_strip[foot].color.b = 146.0/255;
                original_line_strip[foot].color.a = 1.0;

                adaption_line_strip[foot].header.frame_id = "map";
                adaption_line_strip[foot].ns = "ad_lines_ns";
                adaption_line_strip[foot].action = visualization_msgs::Marker::ADD;
                adaption_line_strip[foot].pose.orientation.w = 1.0;
                adaption_line_strip[foot].id = foot;
                adaption_line_strip[foot].type = visualization_msgs::Marker::LINE_STRIP;
                adaption_line_strip[foot].scale.x = 0.01;
                adaption_line_strip[foot].color.r = static_cast<float>(0xE3) / static_cast<float>(0XFF);
                adaption_line_strip[foot].color.g = static_cast<float>(0x51) / static_cast<float>(0XFF);
                adaption_line_strip[foot].color.b = static_cast<float>(0x20) / static_cast<float>(0XFF);
                adaption_line_strip[foot].color.a = 1.0;

                history_line_strip[foot].header.frame_id = "map";
                history_line_strip[foot].ns = "hs_lines_ns";
                history_line_strip[foot].action = visualization_msgs::Marker::ADD;
                history_line_strip[foot].pose.orientation.w = 1.0;
                history_line_strip[foot].id = foot;
                history_line_strip[foot].type = visualization_msgs::Marker::LINE_STRIP;
                history_line_strip[foot].scale.x = 0.01;
                history_line_strip[foot].color.r = static_cast<float>(0x0) / static_cast<float>(0XFF);
                history_line_strip[foot].color.g = static_cast<float>(0xe5) / static_cast<float>(0XFF);
                history_line_strip[foot].color.b = static_cast<float>(0x0) / static_cast<float>(0XFF);
                history_line_strip[foot].color.a = 1.0;
  
            }
            

        }
        ~draw_lines() { }
        void callback(const perceptive_control::robotStates::ConstPtr& states) {

            ros::Time stamp = ros::Time::now();
            if((stamp.toSec() - last_update_time1) < 1.0/running_hz) return;

            step++;
            for (int foot = 0; foot < 2; foot++)
            {
                // #1 retrive the from and dest
                grid_map::Position3 initPos{states->swing_foot_init_pos[0 + foot*3],
                                            states->swing_foot_init_pos[1 + foot*3],
                                            states->swing_foot_init_pos[2 + foot*3]};
                grid_map::Position3 original_targetPos{states->nominal_footholds[0 + foot*3],
                                            states->nominal_footholds[1 + foot*3],
                                            states->nominal_footholds[2 + foot*3]};
                // grid_map::Position3 original_targetPos{states->swing_foot_init_pos[0 + foot*3] + 0.2,
                //                             states->swing_foot_init_pos[1 + foot*3],
                //                             states->swing_foot_init_pos[2 + foot*3]};
                
                // #2 compute the trajectory
                FootSwingTrajectory<double> traj;
                traj.setInitialPosition(initPos);
                traj.setFinalPosition(original_targetPos);
                traj.setHeight(default_swing_height);

                original_line_strip[foot].header.stamp = stamp;
                original_line_strip[foot].points.clear();
                // std::cout << "pointx = [";
                for (int i = 0; i <= resolution; i++)
                {
                    geometry_msgs::Point point;
                    double phase = i/static_cast<double>(resolution);
                    traj.computeSwingTrajectoryBezier(phase, states->swing_total_time[foot]);
                    grid_map::Position3 p =  traj.getPosition();
                    point.x = p[0];
                    point.y = p[1];
                    point.z = p[2];
                    original_line_strip[foot].points.push_back(point);
                }
 
                _original_line_pub[foot].publish(original_line_strip[foot]);

               // draw the real swing trajectory
               if(1) {
                    history_line_strip[foot].header.stamp = ros::Time::now();
                    geometry_msgs::Point point;
                    if(foot == 0) {
                        point.x = states->left_foot_pos[0];
                        point.y = states->left_foot_pos[1];
                        point.z = states->left_foot_pos[2];
                    } else {
                        point.x = states->right_foot_pos[0];
                        point.y = states->right_foot_pos[1];
                        point.z = states->right_foot_pos[2];
                    }
                    history_line_strip[foot].points.push_back(point);
                    _history_line_pub[foot].publish(history_line_strip[foot]);

               }
                

                
            }

            last_update_time1 = ros::Time::now().toSec();

        }
        void adaption_callback(const perceptive_control::perceptiveAdapationInfo::ConstPtr& adapation_info_ptr) {
            ros::Time stamp = ros::Time::now();
            if((stamp.toSec() - last_update_time2) < 1.0/running_hz) return;
            
            for (int foot = 0; foot < 2; foot++)
            {
                // #1 retrive the from and dest
                grid_map::Position3 initPos{adapation_info_ptr->swing_foot_init_pos[0 + foot*3],
                                            adapation_info_ptr->swing_foot_init_pos[1 + foot*3],
                                            adapation_info_ptr->swing_foot_init_pos[2 + foot*3]};
                grid_map::Position3 adpation_targetPos{adapation_info_ptr->recommended_footholds[0 + foot*3],
                                            adapation_info_ptr->recommended_footholds[1 + foot*3],
                                            adapation_info_ptr->recommended_footholds[2 + foot*3]};
                
                // #2 compute the trajectory
                FootSwingTrajectory<double> traj;
                traj.setInitialPosition(initPos);
                traj.setFinalPosition(adpation_targetPos);
                traj.setHeight(adapation_info_ptr->height[foot]);

                adaption_line_strip[foot].header.stamp = stamp;
                adaption_line_strip[foot].points.clear();

                for (int i = 0; i <= resolution; i++)
                {
                    geometry_msgs::Point point;
                    double phase = i/static_cast<double>(resolution);
                    traj.computeSwingTrajectoryBezier(phase, adapation_info_ptr->recommended_swing_times[foot]);
                    grid_map::Position3 p =  traj.getPosition();
                    point.x = p[0];
                    point.y = p[1];
                    point.z = p[2];
                    adaption_line_strip[foot].points.push_back(point);
                }

                _adaption_line_pub[foot].publish(adaption_line_strip[foot]);
                

                
            }


            last_update_time2 = ros::Time::now().toSec();
        }
        
        double running_hz, last_update_time1, last_update_time2;
        int step;
        int resolution;
        double default_swing_height;
        visualization_msgs::Marker original_line_strip[2];
        visualization_msgs::Marker adaption_line_strip[2];
        visualization_msgs::Marker history_line_strip[2];
        ros::Subscriber _states_sub;
        ros::Subscriber _adaption_sub;
        ros::Publisher _original_line_pub[2];
        ros::Publisher _adaption_line_pub[2];
        ros::Publisher _history_line_pub[2];
        ros::NodeHandle _nh;

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;
    ROS_INFO("start draw_swing_trajectory node ...");

    draw_lines draw_lines_(nh);
    

    ros::spin();
    return 0;

}