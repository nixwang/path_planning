//
//  map.cpp
//  OctomapRTT
//
//  Created by Daxin Wang on 2018/7/16.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//

#include "../include/map.hpp"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string.h>
#include <octomap/ColorOcTree.h>

using namespace std;

using namespace octomap;
Map::Map(float resolution, point3d bbx_max, point3d bbx_min)
{
    
//    std::vector<octomap::point3d> points = {point3d(0,0,0), point3d(1,1,1), point3d(0.5,0,0), point3d(1,0,0)};
//    for(auto p:points){
//        map_tree_->updateNode(p, true);
//    }
    bbx_max_ = bbx_max;
    bbx_min_ = bbx_min;
    map_tree_->setBBXMax(bbx_max);
    map_tree_->setBBXMin(bbx_min);
    
//    map_tree_->writeBinary("./map.bt");
}
Map::Map(std::string file_name)
{
    map_tree_ = new octomap::OcTree(file_name);
//    bbx_max_ = map_tree_->getBBXMax();
//    bbx_min_ = map_tree_->getBBXMin();
    bbx_max_ = octomap::point3d(40,40,10);
    bbx_min_ = octomap::point3d(0,0,0);
    map_tree_->setBBXMax(bbx_max_);
    map_tree_->setBBXMin(bbx_min_);
}

Map::Map(const octomap_msgs::Octomap& msg)
{
    // map_tree_ = new octomap::OcTree(file_name);
    octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
    map_tree_ = (octomap::OcTree*)aot;
    // map_tree_ = std::make_shared<octomap::OcTree>(*ot);

//    bbx_max_ = map_tree_->getBBXMax();
//    bbx_min_ = map_tree_->getBBXMin();
    bbx_max_ = octomap::point3d(40,40,10);
    bbx_min_ = octomap::point3d(0,0,0);
    map_tree_->setBBXMax(bbx_max_);
    map_tree_->setBBXMin(bbx_min_);
}

Map::Map(const octomap_msgs::Octomap& msg, bool color_)
{
    AbstractOcTree* read_tree = octomap_msgs::msgToMap(msg);
	map_color_tree_ = dynamic_cast<ColorOcTree*>(read_tree);

	std::cout << "walking the tree to get resolution " << std::endl;

	double res = 999;

	int count = 0; 	
	for(ColorOcTree::leaf_iterator it = map_color_tree_->begin_leafs(), end=map_color_tree_->end_leafs(); it!= end; ++it) {
		if(map_color_tree_->isNodeOccupied(*it)){
			count++;
			double size = it.getSize();
			// std::cout << "block "
			// 	<< it.getX() << " " 
			// 	<< it.getZ() << " "
			// 	<< it.getY() << " "
			// 	<< it->getColor() << " "
			// 	<< size << endl; 
		}
	}
    cout << count << endl;

    bbx_max_ = octomap::point3d(2,2,1);
    bbx_min_ = octomap::point3d(0,0,0);
    map_color_tree_->setBBXMax(bbx_max_);
    map_color_tree_->setBBXMin(bbx_min_);
}

Map::~Map()
{
    delete map_tree_;
}
bool Map::isObstacle(octomap::point3d point)
{
    //to do
    octomap::OcTreeNode* node = map_tree_->search(point);
    if(node!=NULL && node->getOccupancy()>0.5){
        return true;
    }
    return false;
}
void Map::mixPathMap(octomap::point3d point, bool is_occupied)
{
    map_tree_->updateNode(point, is_occupied);
}
void Map::writeFile(std::string output_name)
{
    map_tree_->writeBinary(output_name);
}
