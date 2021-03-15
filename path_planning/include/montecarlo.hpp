//
//  octomap_rrt.hpp
//  OctomapRTT
//
//  Created by 王大鑫 on 2018/7/17.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//

#ifndef montecarlo_hpp
#define montecarlo_hpp

#include <vector>
#include <iostream>
#include <octomap/octomap.h>
#include "map.hpp"

using namespace std;
struct Position;    // 二维采样点的位置
struct Position {
    double x;
    double y;
};

class view_planner{
public:
    view_planner(octomap::point3d current_position, Map *map, int max_iter, double search_radius):current_position_(current_position),map_(map),max_iter_(max_iter),search_radius_(search_radius){
        bestGain_ = 10000;
        std::cout << "construct..." << std::endl;
    }
    ~view_planner(){
        std::cout << "unconstruct" << std::endl;
    }

    Position sampling_point(Position cur_point);
    Position run_ViewPlanner(){
        
    }
    double compute_view_gain(Position point);
    
    Position get_best_view_point();

private:
    octomap::point3d current_position_;
    double bestGain_;
    Position best_view_point_;

    Map *map_;
    
    int max_iter_;
    
    double search_radius_;
};

#endif /* rh_montecarlo_hpp */
