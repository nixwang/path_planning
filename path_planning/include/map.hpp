//
//  map.hpp
//  OctomapRTT
//
//  Created by Daxin Wang on 2018/7/16.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//

#ifndef map_hpp
#define map_hpp

#include <vector>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
// #include <string.h>
#include <octomap/ColorOcTree.h>

class Map{
public:
    // Map(float resolution, octomap::point3d bbx_max, octomap::point3d bbx_min);
    // Map(std::string file_name);
    // Map(const octomap_msgs::Octomap& msg);
    Map(const octomap_msgs::Octomap& msg, bool color_);
    ~Map();
    bool isObstacle(octomap::point3d point);
    bool isInfeatureless(octomap::point3d point);
    double getResolution(){ return map_tree_->getResolution();}
    octomap::point3d getBBXMax(){ return bbx_max_; }
    octomap::point3d getBBXMin(){ return bbx_min_; }
    void mixPathMap(octomap::point3d point, bool is_occupied);
    void writeFile(std::string output_name);
    // octomap::ColorOcTreeNode::Color getcolor(octomap::point3d point);

    octomap::ColorOcTree* map_tree_;

private:    

    // octomap::OcTree* map_tree_;

    float resolution_;
    octomap::point3d bbx_max_;
    octomap::point3d bbx_min_;
};
#endif /* map_hpp */
