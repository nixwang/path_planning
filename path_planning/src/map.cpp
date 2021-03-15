//
//  map.cpp
//  OctomapRTT
//
//  Created by Daxin Wang on 2018/7/16.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include "../include/map.hpp"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string.h>
#include <octomap/ColorOcTree.h>

using namespace std;

using namespace octomap;
// Map::Map(float resolution, point3d bbx_max, point3d bbx_min)
// {
    
// //    std::vector<octomap::point3d> points = {point3d(0,0,0), point3d(1,1,1), point3d(0.5,0,0), point3d(1,0,0)};
// //    for(auto p:points){
// //        map_tree_->updateNode(p, true);
// //    }
//     bbx_max_ = bbx_max;
//     bbx_min_ = bbx_min;
//     map_tree_->setBBXMax(bbx_max);
//     map_tree_->setBBXMin(bbx_min);
    
// //    map_tree_->writeBinary("./map.bt");
// }
// Map::Map(std::string file_name)
// {
//     map_tree_ = new octomap::OcTree(file_name);
// //    bbx_max_ = map_tree_->getBBXMax();
// //    bbx_min_ = map_tree_->getBBXMin();
//     bbx_max_ = octomap::point3d(40,40,10);
//     bbx_min_ = octomap::point3d(0,0,0);
//     map_tree_->setBBXMax(bbx_max_);
//     map_tree_->setBBXMin(bbx_min_);
// }

// Map::Map(const octomap_msgs::Octomap& msg)
// {
//     // map_tree_ = new octomap::OcTree(file_name);
//     octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
//     map_tree_ = (octomap::OcTree*)aot;
//     // map_tree_ = std::make_shared<octomap::OcTree>(*ot);

// //    bbx_max_ = map_tree_->getBBXMax();
// //    bbx_min_ = map_tree_->getBBXMin();
//     bbx_max_ = octomap::point3d(40,40,10);
//     bbx_min_ = octomap::point3d(0,0,0);
//     map_tree_->setBBXMax(bbx_max_);
//     map_tree_->setBBXMin(bbx_min_);
// }

Map::Map(const octomap_msgs::Octomap& msg, bool color_)
{
    AbstractOcTree* read_tree = octomap_msgs::msgToMap(msg);
	map_tree_ = dynamic_cast<ColorOcTree*>(read_tree);

	std::cout << "walking the tree to get resolution " << std::endl;

	// double res = 999;

	// int count = 0; 	
	// for(ColorOcTree::leaf_iterator it = map_color_tree_->begin_leafs(), end=map_color_tree_->end_leafs(); it!= end; ++it) {
	// 	if(map_color_tree_->isNodeOccupied(*it)){
	// 		count++;
	// 		double size = it.getSize();
	// 		// std::cout << "block "
	// 		// 	<< it.getX() << " " 
	// 		// 	<< it.getZ() << " "
	// 		// 	<< it.getY() << " "
	// 		// 	<< it->getColor() << " "
	// 		// 	<< size << endl; 
	// 	}
	// }
    // cout << count << endl;

    // double max_x,max_y,max_z;
    // double min_x,min_y,min_z;
    // map_tree_->getMetricMax(max_x,max_y,max_z);
    // cout<<max_x<<" "<<max_y<<" "<<max_z<<endl;
    // map_tree_->getMetricMin(min_x,min_y,min_z);
    // cout<<min_x<<" "<<min_y<<" "<<min_z<<endl;
    // octomap::point3d max_p(max_x, max_y, max_z);
    // octomap::point3d min_p(min_x, min_y, min_z);
    // cout<<max_p.x()<<" "<<max_p.y()<<" "<<max_p.z()<<endl;
    // map_tree_->setBBXMax(max_p);
    // map_tree_->setBBXMin(min_p);

    // 5 0.6 2.6
    // -0.2 0 -2.6

    bbx_max_ = octomap::point3d(50,1,50);
    bbx_min_ = octomap::point3d(-50,-1,-50);
    map_tree_->setBBXMax(bbx_max_);
    map_tree_->setBBXMin(bbx_min_);
}

Map::~Map()
{
    delete map_tree_;
}
bool Map::isObstacle(octomap::point3d point)
{
    //to do
    octomap::ColorOcTreeNode* node = map_tree_->search(point);
    if(node!=NULL && node->getOccupancy()>0.5){
        cout << "searched node Occupancy" << node->getOccupancy() << endl;

        cout << "searched node color" << node->getColor() << endl;
        return true;
    }
    return false;
}

bool Map::isInfeatureless(octomap::point3d point)
{
    double y_s = point.y();
    for(int i = 0; i <= 5; ++i){
        double y_hight = i*0.2 + 0.1;

        point.y() = y_hight;
        octomap::ColorOcTreeNode* node = map_tree_->search(point);
    
        if(node!=NULL && node->getOccupancy()>0.5){
            ROS_WARN("searched node position: %f %f %f", point.x(), point.y(), point.z());
            ROS_WARN("q_new_gain_color: %d %d %d", node->getColor().r,node->getColor().g, node->getColor().b);

            if((node->getColor().r == 255) && (node->getColor().g == 255) && (node->getColor().b == 255)){
                continue;
            }

            if((node->getColor().r <=(61+5)) && (node->getColor().r >= (61-5)) && (node->getColor().g <=(230+5)) && (node->getColor().g >= (230-5)) && (node->getColor().b <=(250+5)) && (node->getColor().b >= (250-5))){
                ////// 在无效定位区域内，则增益为1000.因为最后最优增益是求最小值
                ROS_ERROR("Invalid area!!!");
                // gain = -100;
                point.y() = y_s;
                return true;
            }
            else{
                ///// todo: 不在无效定位区域内，则计算距离
                // gain = 50;
                point.y() = y_s;
                return false;
            }

            // if(node->getColor())
            break;
        }
    }

    /***可用来表征未建图区域**********************************************/
    point.y() = y_s;
    return false;     // 若在未知区域，返回0
}

// ColorOcTreeNode::Color Map::getcolor(octomap::point3d point)
// {
//     //to do
//     octomap::ColorOcTreeNode* node = map_tree_->search(point);
//     if(node!=NULL){
//         return node->getColor();
//     }
//     // return false;
// }


void Map::mixPathMap(octomap::point3d point, bool is_occupied)
{
    map_tree_->updateNode(point, is_occupied);
}
void Map::writeFile(std::string output_name)
{
    map_tree_->writeBinary(output_name);
}
