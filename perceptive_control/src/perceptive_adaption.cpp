/*
 * @Author: haoyun 
 * @Date: 2023-05-17 14:37:53
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-19 09:18:07
 * @FilePath: /elevation_map_ws/src/perceptive_control/src/perceptive_adaption.cpp
 * @Description: foothold adaption + lift height iteration
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <vector>

#include "exlcm/example_t.hpp"
#include "perceptionlcm/robot_states.hpp"
#include "perceptionlcm/perception_information.hpp"
#include <lcm/lcm-cpp.hpp>
// Grid Map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// Perceptive control
#include "perceptive_control/robotStates.h"
#include "perceptive_control/perceptiveAdapationInfo.h"

// Swing trajectory
#include "trajectory_planning/FootSwingTrajectory.h"


class perceptive_adaption
{
 public:
    perceptive_adaption(ros::NodeHandle &nh)
    :_nh(nh),_lcm_send_handle("udpm://239.255.76.67:7667?ttl=1")
    { }
    ~perceptive_adaption() {}
    void init() {
        ROS_INFO("start to initilize the perceptive adaption node ...");
        update_map = true;
        receive_map = false;
        running_hz = 1000;
        height_offset.resize(2);
        height_offset[0] = 0.0;
        height_offset[1] = 0.0;
       
        _default_swing_height = 0.08; // 8 cm 
        _max_height = 0.15;
        last_update_time = ros::Time::now().toSec();
        _states_sub =  _nh.subscribe("/elevation_mapping/elevation_map_raw", 1, &perceptive_adaption::map_callback, this);
        _elevation_sub = _nh.subscribe<perceptive_control::robotStates>("robot_states", 1, &perceptive_adaption::state_callback, this);
        
        this->_perceptive_info_pub = this->_nh.advertise<perceptive_control::perceptiveAdapationInfo>(
            "adaption_info", 10    
        );

        perceptive_msg.pitch_ref = 0.0;
        // _lcm_send_handle.publish("PERCEPTIVE_CONTROL_MSG", &perceptive_msg);
        
    }
    void map_callback(const grid_map_msgs::GridMap& message) {
        // ROS_INFO("revice elevation");
        if(update_map) {
            // map_raw.clearAll();
            if(grid_map::GridMapRosConverter::fromMessage(message, map_raw)) 
            {
                
                receive_map = true;
                
            }       
            else
                ROS_INFO("map update fail");  
            
        }
    }

    void state_callback(const perceptive_control::robotStates::ConstPtr& states) {

        // map_raw.setPosition(grid_map::Position(states->position[0], states->position[1]));
        // grid_map::Position pos;
        // map_raw.getPosition(grid_map::Index(0, 0), pos);
        // std::cout << "pos = " << pos.transpose() << std::endl;
        // std::cout << "pos - base pos = " << pos.transpose() - grid_map::Position(states->position[0], states->position[1]).transpose() << std::endl;
        // grid_map::Index index;
        // map_raw.getIndex(grid_map::Position(states->position[0], states->position[1]), index);
        // ROS_INFO_STREAM("position of base:" << grid_map::Position(states->position[0], states->position[1]).transpose());
        // ROS_INFO_STREAM("position of map:" << map_raw.getPosition().transpose());
        
        // ROS_INFO_STREAM("base index = " << index(0) << ",  = " << index(1));
        double now = ros::Time::now().toSec();
        // ROS_INFO("State_callback");
        // std::cout << "now - last_update_time = " << (now - last_update_time)/1e3 << std::endl;
        // if((now - last_update_time)/1e3 > 1.0/running_hz)
    
        if(receive_map)
        // if(1)
        {
            // ROS_INFO("entering perceptive control");
            if(!receive_map) {
            ROS_INFO("Perceptive controller has not received elevation map.");
            update_map = true;
            return ;
            }
            
            update_map = false;
            float radius = 0.1;
            float valid_threshold = 0.8;
            
            for (int leg = 0; leg < 2; leg++)
            {
                if(states->plan_swings[leg] > 0.6) break; // lock the target position when 

                grid_map::Position3 initPos{states->swing_foot_init_pos[0 + leg*3],
                                            states->swing_foot_init_pos[1 + leg*3],
                                            states->swing_foot_init_pos[2 + leg*3]};
                // std::cout << "init pos = " <<  initPos.transpose() << std::endl;                     

                // grid_map::Position init_pos_in_local(initPos[0] - states->position[0],
                //                                      initPos[1] - states->position[1]); // express in the local frame
                grid_map::Position init_pos_in_local(initPos[0],
                                                     initPos[1]); // express in the local frame
        
                // std::cout << "local init pos = " <<  init_pos_in_local.transpose() << std::endl;  

                int k = 0;
                height_offset[leg] = 0.0;
                // ROS_INFO("so far so good1");
                grid_map::Matrix& data = map_raw["elevation_inpainted"];
                double radius_init = 0.02;
                
                for(grid_map::CircleIterator iterator(map_raw, init_pos_in_local, radius_init);
                    !iterator.isPastEnd(); ++iterator) {

                    //  std::cout << "checking... " << std::endl;  
                     grid_map::Index index(*iterator);
                    //  std::cout << "index = " << index(0) <<", "<< index(1) << std::endl;
                     grid_map::Position pos;
                     map_raw.getPosition(*iterator, pos);
                    //  std::cout << "pos = " << pos.transpose() << std::endl;

                    if(!map_raw.isValid(index, "elevation_inpainted")) {
                        // std::cout << "invalid!" << std::endl;
                        // std::cout << "look invalid elevation = " << data(index(0), index(1)) << std::endl;
                        continue;
                    }
                        // std::cout << "we find valid elevation = " << std::endl;
                    // mean offset
                   
                    // std::cout << "find valid elevation = " << data(index(0), index(1)) << std::endl;
                    k++;
                    height_offset[leg] += 1.0/static_cast<double>(k) * (data(index(0), index(1)) - height_offset[leg]);
                }
                // std::cout << "k =" << k << std::endl;
                for (int i = 0; i < 3; i++) {
                // default position expressed in the world frame
                    perceptive_msg.recommended_footholds[i + leg*3] = states->nominal_footholds[i + leg*3];
                }
                

                if(k < 1) {
                    // no enough surrounding information of the initial position
                    // then use the original target foothold and break
                    perceptive_msg.height[leg] = _default_swing_height;
                    ROS_INFO("Leg %d is missing the surrounding information, use default target foothold", leg);
                    perceptive_msg.recommended_swing_times[leg] = states->swing_total_time[leg];

                } else {
                    // Step #1: start local search
                    bool foothold_change = false;
                    // grid_map::Position center(perceptive_msg.recommended_footholds[0 + leg*3] - states->position[0],
                    //                             perceptive_msg.recommended_footholds[1 + leg*3] - states->position[1]);
                    grid_map::Position center(perceptive_msg.recommended_footholds[0 + leg*3],
                                                perceptive_msg.recommended_footholds[1 + leg*3]);
                    grid_map::Matrix& data2 = map_raw["traversability"];
                    for(grid_map::SpiralIterator iterator(map_raw, center, radius); !iterator.isPastEnd(); ++iterator) {
                        if(!map_raw.isValid(*iterator, "traversability")) continue;
                        grid_map::Index index(*iterator);
                        if(data2(index(0), index(1)) >= valid_threshold) {
                            grid_map::Position valid_foothold;
                            map_raw.getPosition(*iterator, valid_foothold);
                            // ROS_INFO_STREAM("leg" << leg <<  ": before target = [" << perceptive_msg.recommended_footholds[0 + leg*3] << ", " << perceptive_msg.recommended_footholds[1 + leg*3] << "] elevation = " << perceptive_msg.recommended_footholds[2 + leg*3]);
                            // perceptive_msg.recommended_footholds[0 + leg*3] = states->position[0] + valid_foothold[0];
                            // perceptive_msg.recommended_footholds[1 + leg*3] = states->position[1] + valid_foothold[1];
                            // perceptive_msg.recommended_footholds[2 + leg*3] = map_raw.at("elevation_inpainted", index) - height_offset[leg];
                            perceptive_msg.recommended_footholds[0 + leg*3] = valid_foothold[0];
                            perceptive_msg.recommended_footholds[1 + leg*3] = valid_foothold[1];
                            perceptive_msg.recommended_footholds[2 + leg*3] = map_raw.at("elevation_inpainted", index) - height_offset[leg];
                            foothold_change = true;

                            // ROS_INFO_STREAM("leg" << leg <<  ": find optimal position. posInWolrd = " << perceptive_msg.recommended_footholds[0 + leg*3] << ", " << perceptive_msg.recommended_footholds[1 + leg*3] << " elevation = " << perceptive_msg.recommended_footholds[2 + leg*3]);
                                
                            break;
                        }
                    }
                    if (!foothold_change)
                    {
                        ROS_INFO("leg %d fails to find a feasible foothold, use defualt target foothold", leg);
                    }
                    // Step #1.5 evaluate the swing time
                    perceptive_msg.recommended_swing_times[leg] = states->swing_total_time[leg];

                    // Step #2: iterate the height
                    
                                                
                    grid_map::Position3 targetPos{perceptive_msg.recommended_footholds[0 + leg*3],
                                                perceptive_msg.recommended_footholds[1 + leg*3],
                                                perceptive_msg.recommended_footholds[2 + leg*3]};

                    
                    perceptive_msg.height[leg] = _default_swing_height;
                    grid_map::Position3 base_pose;
                    base_pose << states->position[0], states->position[1], states->position[2]; 
                    perceptive_msg.height[leg] = iterateTheHeight(initPos, targetPos, base_pose, perceptive_msg.recommended_swing_times[leg],
                                            states->plan_swings[leg], perceptive_msg.height[leg], height_offset[leg]);

                }


                

                
                // Step #3: argmin the pitch angle


            }

            // float height = this->map_raw.atPosition("elevation_inpainted", grid_map::Position(states->position[0], states->position[1]));
            // if(height > 0.02)
            //     std::cout << "now the elevation is" << height << std::endl;

            // send the lcm
            // ROS_INFO("send lcm");
            _lcm_send_handle.publish("PERCEPTIVE_CONTROL_MSG", &perceptive_msg);


            // publish the corresponding topic
            perceptive_adapation_info.pitch_ref = perceptive_msg.pitch_ref;
            for (int i = 0; i < 2; i++)
            {
                perceptive_adapation_info.recommended_footholds[i] = perceptive_msg.recommended_footholds[i];
                perceptive_adapation_info.swing_foot_init_pos[i] = states->swing_foot_init_pos[i];
                perceptive_adapation_info.recommended_swing_times[i] = perceptive_msg.recommended_swing_times[i];
                perceptive_adapation_info.height[i] = perceptive_msg.height[i];
            }
            for (int i = 2; i < 6; i++)
            {
                perceptive_adapation_info.recommended_footholds[i] = perceptive_msg.recommended_footholds[i];
                perceptive_adapation_info.swing_foot_init_pos[i] = states->swing_foot_init_pos[i];
            }
            // ROS_INFO("publish adaption");
            this->_perceptive_info_pub.publish(perceptive_adapation_info);
            
        
            update_map = true;
            // receive_map = false;
            last_update_time = ros::Time::now().toSec();
        }
    }

    double iterateTheHeight(
        grid_map::Position3 from,
        grid_map::Position3 to,
        grid_map::Position3 basePos,
        double swing_time,
        double phase,
        double init_height,
        double offset
    ) {
        FootSwingTrajectory<double> check_trajectory;
        double valid_height = init_height;
        bool valid_flag = false;
        check_trajectory.setInitialPosition(from);
        check_trajectory.setFinalPosition(to);

        // collision check
        while(!valid_flag) {
            valid_flag = true;
            check_trajectory.setHeight(valid_height);
            int seed = static_cast<int>(phase * 10); 
            seed = (seed<3)?3:((seed>8)?8:seed);
            for (; seed < 8; seed++)
            {
                double check_phase = static_cast<double>(seed)/10.0;
                check_trajectory.computeSwingTrajectoryBezier(check_phase, swing_time);
                Eigen::Matrix<double, 3, 1> check_pos =  check_trajectory.getPosition();
                // grid_map::Position pos_in_local(check_pos[0] - basePos[0],
                //                             check_pos[1] - basePos[1]); // express in the local frame
                grid_map::Position pos_in_local(check_pos[0],
                                            check_pos[1]); // express in the local frame

                double err = this->map_raw.atPosition("elevation_inpainted", pos_in_local) - offset - check_pos[2];
                // std::cout << "seed = " << seed 
                //             <<"phase =" << check_phase << " ,plan height = " << check_pos[2] << " ,elevation height = "
                //             << this->map_raw.atPosition("elevation_inpainted", pos_in_local) - offset
                //             << "err = " << err 
                //             << "cureent height =" << valid_height << std::endl;
                if(err > -0.02) {
                    valid_flag = false;
                    valid_height += (0.1 * (err + 0.02) + 0.01);
                    if(valid_height >= _max_height) {
                        valid_height = _max_height;
                        valid_flag = true;
                        ROS_WARN("leg Collision check fails. use the maximum swing foot's height");
                    }
                    break;
                }
            }
            
        }

        return valid_height;

    }


    ros::NodeHandle _nh;
    ros::Publisher _perceptive_info_pub;
    perceptive_control::perceptiveAdapationInfo perceptive_adapation_info;
    lcm::LCM _lcm_send_handle;
    perceptionlcm::perception_information perceptive_msg;
    bool update_map;
    bool receive_map;
    grid_map::GridMap map_raw;
    double last_update_time;
    double running_hz;
    double _default_swing_height;
    double _max_height;
    std::vector<double> height_offset;
    ros::Subscriber _states_sub;
    ros::Subscriber _elevation_sub;

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "perceptive_control_node_init_name");
    ros::NodeHandle nh;

    perceptive_adaption perceptiveController(nh);
    perceptiveController.init();

    // lcm::LCM _lcm_send_handle();

    ros::spin();
    return 0;
}