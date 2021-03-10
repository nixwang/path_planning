//
//  octomap_rrt.cpp
//  OctomapRTT
//
//  Created by 王大鑫 on 2018/7/17.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//

#include "../include/octomap_rrt.hpp"
#include <chrono>
#include <fstream>
RRT3D::RRT3D(octomap::point3d start_position, octomap::point3d end_position, Map *map, int max_iter, short step_size)
{
    start_position_ = start_position;
    end_position_ = end_position;
    // gain_ = gain;
    map_ = map;
    root_ = new Node;
    root_->parent = NULL;
    root_->position = start_position_;
    // root_->gain = gain_;
    nodes_.push_back(root_);
    lastNode_ = root_;
    max_iter_ = max_iter;
    step_size_ = step_size;
    std::cout << "step_size = " << step_size_ << std::endl;
}

RRT3D::~RRT3D()
{
    deleteNodes(root_);
}
void RRT3D::deleteNodes(Node *root)
{
    for(auto node:root->children) {
        deleteNodes(node);
    }
    delete root;
}

Node* RRT3D::getRandomNotObstacleNode()
{
    octomap::point3d rand_point;
    std::cout<<"enter run " << map_->getBBXMax().x() <<std::endl;
    std::cout<<"enter run " << map_->getBBXMax().y() <<std::endl;
    std::cout<<"enter run " << map_->getBBXMax().z() <<std::endl;
    short x_max = map_->getBBXMax().x()/ map_->getResolution();
    // std::cout<<"enter run 0.1" <<std::endl;
    short y_max = map_->getBBXMax().y()/ map_->getResolution();
    short z_max = map_->getBBXMax().z()/ map_->getResolution();

    do{
        rand_point = octomap::point3d(rand()%x_max*map_->getResolution(),
                                     rand()%y_max*map_->getResolution(),
                                     rand()%z_max*map_->getResolution());
        cout <<"rand_point:" << rand_point.x() << " " << rand_point.y() << " " <<  rand_point.z() << endl;

    }while(map_->isObstacle(rand_point));
    Node *rand_node = new Node;
    rand_node->position = rand_point;
    return rand_node;
}

Node* RRT3D::findNearestNode(octomap::point3d current_position)
{
    double min_distance = 1e5;
    Node* closest_node = NULL;
    for(auto node:nodes_){
        double distance = (current_position - node->position).norm();
        if(distance<min_distance){
            min_distance = distance;
            closest_node = node;
        }
    }
    return closest_node;
}

Node* RRT3D::getNewNode(Node *q_rand, Node *q_nearest, octomap::point3d direction)
{
    octomap::point3d result;
    result.x()= q_nearest->position.x() + map_->getResolution()*step_size_*direction.x();
    result.y()= q_nearest->position.y() + map_->getResolution()*step_size_*direction.y();
    result.z()= q_nearest->position.z() + map_->getResolution()*step_size_*direction.z();
    Node *q_new = new Node;
    q_new->position = result;
    return q_new;
}

bool RRT3D::isNewNodeCollision(octomap::point3d q_new, octomap::point3d q_nearest,  octomap::point3d direction)
{
    int test_step = 4;
    bool ret = false;
    for(int i=1; i<=test_step; i++){
        float step = 1.0*i/test_step*step_size_;
        octomap::point3d test_point;
        test_point.x()= q_nearest.x() + map_->getResolution()*step*direction.x();
        test_point.y()= q_nearest.y() + map_->getResolution()*step*direction.y();
        test_point.z()= q_nearest.z() + map_->getResolution()*step*direction.z();
        if(map_->isObstacle(test_point)){
            ret = true;
            break;
        }
    }
    return ret;
}

double RRT3D::compute_gain(Node *q_new)
{
    double gain = 0;
    
    // gain分为与目标点的距离+与边界的距离
    // int count = 0; 	
	// for(octomap::ColorOcTree::leaf_iterator it = map_->map_color_tree_->begin_leafs(), end=map_->map_color_tree_->end_leafs(); it!= end; ++it) {
	// 	if(map_->map_color_tree_->isNodeOccupied(*it)){
	// 		count++;
	// 		double size = it.getSize();
	// 		std::cout << "block "
	// 			<< it.getX() << " " 
	// 			<< it.getZ() << " "
	// 			<< it.getY() << " "
	// 			<< it->getColor() << " "
	// 			<< size << endl; 
	// 	}
	// }
    // cout << count << endl;
    
    // q_new->gain = q_new->parent->gain + gain;
    return gain;
}

void RRT3D::addNewNode(Node *q_nearest, Node *q_new)
{
    q_new->parent = q_nearest;
    q_nearest->children.push_back(q_new);
    nodes_.push_back(q_new);
    lastNode_ = q_new;
//    std::cout<<"Get a new Node"<<std::endl;
//    std::cout<<"Now nodes have:"<<nodes_.size()<<std::endl;
//    for(auto node:nodes_){
//        std::cout<<node->position<<"\n";
//    }
//    std::cout<<"\n";
}

bool RRT3D::isArrived()
{
//    if((lastNode_->position - end_position_).norm() < 2.2*map_->getResolution())
//        return true;
    if((lastNode_->position - end_position_).norm() < 5*map_->getResolution())
        return true;
    return false;
}

bool RRT3D::run(bool debug)
{
    srand(static_cast<ushort>(time(NULL)));
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Node *q_rand = getRandomNotObstacleNode();

    for(int i=0; i<max_iter_; i++){
        if(debug)
            if(i%100==0)
                std::cout<<"i="<<i<<std::endl;
        Node *q_rand = getRandomNotObstacleNode();
//        std::cout<<"random_point: "<<q_rand->position<<std::endl;
        Node *q_nearest = findNearestNode(q_rand->position);
//        std::cout<<"nearest_node: "<<q_nearest->position<<std::endl;
        octomap::point3d direction = q_rand->position - q_nearest->position;
        // std::cout<<"enter run 3" <<std::endl;
        if(direction.norm() > step_size_){
            direction = direction.normalize();
            Node *q_new = getNewNode(q_rand, q_nearest, direction);
            if(!isNewNodeCollision(q_new->position, q_nearest->position, direction))
                addNewNode(q_nearest, q_new);
        }
        // std::cout<<"enter run 4" <<std::endl;
        if(isArrived()){
            break;
        }
    }

    Node *q;
    if(isArrived()){
        q = lastNode_;
        std::cout<<"Find a path\n";
    }
    else{
        q = findNearestNode(end_position_);
        std::cout<<"Can not find the path\n";
    }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
    while(q!=NULL){
        path_.push_back(q);
        q = q->parent;
    }

    return true;
}

void RRT3D::writeMap()
{
    //visualize path by writing path into map module
    for(auto node: path_){
        std::cout<<node->position<<std::endl;
        map_->mixPathMap(node->position, true);
    }
}
void RRT3D::writeInfo2File(std::string output_name)
{
    double distance = 0;
    double tmp_distance = 0;
    ofstream fout;
    fout.open(output_name);
    //write basic infomation to file
    fout<<"step_size = "<<step_size_<<std::endl;
    fout<<"max_iter = "<<max_iter_<<std::endl;
    fout<<"start_position = "<<start_position_<<"\tend_position = "<<end_position_<<std::endl;
    //write position of path-node and distance between two nodes 
    fout<<"START_POSITION\t\t\t"<<"END_POSITION\t\t\t"<<"DISTANCE\t"<<"TOTAL_DISTANCE\n";
    for (int i=path_.size()-1; i>0; i--){
        tmp_distance = (path_[i]->position - path_[i-1]->position).norm();
        fout<<path_[i]->position<<"\t"<<path_[i-1]->position<<"\t"<<tmp_distance<<"\t";
        distance += tmp_distance;
        fout<<distance<<std::endl;
    }
    //write distance between last_node_position and end_position 
    fout<<"LAST_NODE_POSITION\t\t\t"<<"FINAL_POSITION\t\t\t"<<"DISTANCE\t"<<"TOTAL_DISTANCE\n";
    tmp_distance = (end_position_ - path_[0]->position).norm();
    fout<<path_[0]->position<<"\t"<<end_position_<<"\t"<<tmp_distance<<"\t";
    distance += tmp_distance;
    fout<<distance<<std::endl;

    std::cout<<"distance = "<<distance<<std::endl;
    fout<<flush;
    fout.close();
}