//
//  octomap_rrt.cpp
//  OctomapRTT
//
//  Created by 王大鑫 on 2018/7/17.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include "../include/montecarlo.hpp"

#include <chrono>
#include <fstream>

// view_planner::view_planner(octomap::point3d current_position, Map *map, int max_iter, double search_radius)
// {
//     bestGain_ = 10000;
// }

// view_planner::~view_planner()
// {
//     // deleteNodes(root_);
//     std::cout << "unconstruct" << std::endl;
// }

// 这个函数完成视角落点的采样过程，限制在采样半径内
Position view_planner::sampling_point(Position cur_point){
    Position sample_point;
    
    /*
        coding here..........
    */

    return sample_point;
}

// 这个函数完成每个采样点的信息增益的计算
double view_planner::compute_view_gain(Position point){
    double gain = 0;

    /*
        coding here..........
    */
   
    return gain;
}


//////////////可用于参考/////////////////
// double RRT3D::border_gain(Node *q_new){
//     double gain; 

//     double y_s = q_new->position.y();
//     for(int i = 0; i <= 5; ++i){
//         double y_hight = i*0.2 + 0.1;

//         q_new->position.y() = y_hight;
//         octomap::ColorOcTreeNode* node = map_->map_tree_->search(q_new->position);
    
//         if(node!=NULL && node->getOccupancy()>0.5){
//             ROS_WARN("searched node position: %f %f %f", q_new->position.x(), q_new->position.y(), q_new->position.z());
//             ROS_WARN("q_new_gain_color: %d %d %d", node->getColor().r,node->getColor().g, node->getColor().b);

//             if((node->getColor().r == 255) && (node->getColor().g == 255) && (node->getColor().b == 255)){
//                 continue;
//             }

//             if((node->getColor().r <=(61+5)) && (node->getColor().r >= (61-5)) && (node->getColor().g <=(230+5)) && (node->getColor().g >= (230-5)) && (node->getColor().b <=(250+5)) && (node->getColor().b >= (250-5))){
//                 ////// 在无效定位区域内，则增益为1000.因为最后最优增益是求最小值
//                 ROS_ERROR("Invalid area!!!");
//                 gain = -100;
//                 q_new->position.y() = y_s;
//                 return gain;
//             }
//             else{
//                 ///// todo: 不在无效定位区域内，则计算距离
//                 gain = 50;
//                 q_new->position.y() = y_s;
//                 return gain;
//             }

//             // if(node->getColor())
//             break;
//         }
//     }

//     /***可用来表征未建图区域**********************************************/
//     q_new->position.y() = y_s;
//     return 0;     // 若在未知区域，返回0
// }

// double RRT3D::compute_gain(Node *q_new)
// {
//     double gain = 0;
//     if(q_new->parent==NULL){
//         return gain;
//     }    

//     cout << "q_new->parent: " << q_new->parent->position.x() << " " << q_new->parent->position.y()<< " " << q_new->parent->position.z() << endl;
    
//     // gain分为与目标点的距离+与边界的距离

//     double gain_border_distance = border_gain(q_new);
//     ROS_INFO("gain_border_distance: %f", gain_border_distance);

//     double parent_distance;
//     parent_distance = (q_new->parent->position - end_position_).norm();

//     double current_distance;
//     current_distance = (q_new->position - end_position_).norm();

//     double gain_goal_distance = current_distance - parent_distance;
//     ROS_INFO("gain_goal_distance: %f", gain_goal_distance);


//     // gain = q_new->parent->gain - 10*gain_goal_distance - gain_border_distance;
    
//     if(q_new->parent!=NULL){
//         gain = 5*gain_goal_distance - gain_border_distance;
//         ROS_INFO("node_gain: %f", gain);
//     }
//     else{
//         gain = 5*gain_goal_distance - gain_border_distance;
//         ROS_INFO("root_node_gain: %f", gain);
//     }
    
//     return gain;
// }


Position view_planner::run_ViewPlanner()
{
    srand(static_cast<ushort>(time(NULL)));
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for(int i=0; i<max_iter_; i++){
        
        std::cout<<"i="<<i<<std::endl;
        Position sample_point = sampling_point(current_position_);
        ROS_INFO("sample_point: %f %f", sample_point.x, sample_point.y);

        double point_gain = compute_gain(sample_point);
        ROS_INFO("point_gain: %f", point_gain);

        if (point_gain < bestGain_) {   // 或大于：具体看增益函数
            ROS_INFO("bestGain_: %f", bestGain_);
            bestGain_ = point_gain;
            best_view_point_.x = sample_point.x;
            best_view_point_.y = sample_point.y;
        }
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";

    return best_view_point_;

}
