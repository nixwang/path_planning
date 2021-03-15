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
    root_->gain = 0;
    nodes_.push_back(root_);
    lastNode_ = root_;
    max_iter_ = max_iter;
    step_size_ = step_size;
    bestGain_ = 10000;
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
    float prob = 0.5;
    float random_f;
    int N = 999;
    random_f = rand() % (N + 1) / (float)(N + 1);
    if(random_f >= prob){   // 大于一定概率，则进行随机采样；否则向着目标点采样
        octomap::point3d rand_point;
        // ROS_INFO("getBBXMax: %f %f %f", map_->getBBXMax().x(), map_->getBBXMax().y(), map_->getBBXMax().z());
        // cout << "map_->getResolution() " << map_->getResolution() << endl;
        short x_max = map_->getBBXMax().x()/ map_->getResolution();
        short y_max = map_->getBBXMax().y()/ map_->getResolution();
        short z_max = map_->getBBXMax().z()/ map_->getResolution();

        short x_min = map_->getBBXMin().x()/ map_->getResolution();
        short y_min = map_->getBBXMin().y()/ map_->getResolution();
        short z_min = map_->getBBXMin().z()/ map_->getResolution();

        // do{
        //     rand_point = octomap::point3d(rand()%x_max*map_->getResolution(),
        //                                  rand()%y_max*map_->getResolution(),
        //                                  rand()%z_max*map_->getResolution());
        //     // rand_point = octomap::point3d(rand()%x_max*map_->getResolution(),
        //     //                              rand()%y_max*map_->getResolution(),
        //     //                              0.5);
        // }while(map_->isObstacle(rand_point));

        // ROS_INFO("x_max: %d %d %d", x_max, y_max, z_max);
        do{
            rand_point = octomap::point3d(((rand()%(x_max-x_min))+x_min)*map_->getResolution(),
                                        0.3,
                                    ((rand()%(z_max-z_min))+z_min)*map_->getResolution());
        }while(map_->isInfeatureless(rand_point));
        

        // ROS_INFO("rand_point: %f %f %f", rand_point.x(), rand_point.y(), rand_point.z());

        Node *rand_node = new Node;
        rand_node->position = rand_point;
        return rand_node;
    }
    else{
        Node *rand_node = new Node;
        rand_node->position = end_position_;

        return rand_node;
    }
}

Node* RRT3D::findNearestNode(octomap::point3d current_position)
{
    double min_distance = 1e5;
    Node* closest_node = NULL;
    for(auto node:nodes_){
        double distance = (current_position - node->position).norm();
        if(distance < min_distance){
            min_distance = distance;
            closest_node = node;
        }
    }

    ROS_INFO("closest_node: %f %f %f", closest_node->position.x(), closest_node->position.y(), closest_node->position.z());
    return closest_node;
}

Node* RRT3D::getNewNode(Node *q_rand, Node *q_nearest, octomap::point3d direction)
{
    octomap::point3d result;

    // ROS_INFO("q_nearest: %f %f %f", q_nearest->position.x(), q_nearest->position.y(), q_nearest->position.z());
    // ROS_INFO("map_->getResolution(): %f", map_->getResolution());
    // ROS_INFO("step_size_: %d", step_size_);
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
        if(map_->isInfeatureless(test_point)){
            ret = true;
            break;
        }
    }
    return ret;
}

double RRT3D::border_gain(Node *q_new){
    double gain; 

    double y_s = q_new->position.y();
    for(int i = 0; i <= 5; ++i){
        double y_hight = i*0.2 + 0.1;

        q_new->position.y() = y_hight;
        octomap::ColorOcTreeNode* node = map_->map_tree_->search(q_new->position);
    
        if(node!=NULL && node->getOccupancy()>0.5){
            ROS_WARN("searched node position: %f %f %f", q_new->position.x(), q_new->position.y(), q_new->position.z());
            ROS_WARN("q_new_gain_color: %d %d %d", node->getColor().r,node->getColor().g, node->getColor().b);

            if((node->getColor().r == 255) && (node->getColor().g == 255) && (node->getColor().b == 255)){
                continue;
            }

            if((node->getColor().r <=(61+5)) && (node->getColor().r >= (61-5)) && (node->getColor().g <=(230+5)) && (node->getColor().g >= (230-5)) && (node->getColor().b <=(250+5)) && (node->getColor().b >= (250-5))){
                ////// 在无效定位区域内，则增益为1000.因为最后最优增益是求最小值
                ROS_ERROR("Invalid area!!!");
                gain = -100;
                q_new->position.y() = y_s;
                return gain;
            }
            else{
                ///// todo: 不在无效定位区域内，则计算距离
                gain = 50;
                q_new->position.y() = y_s;
                return gain;
            }

            // if(node->getColor())
            break;
        }
    }

    /***可用来表征未建图区域**********************************************/
    q_new->position.y() = y_s;
    return 0;     // 若在未知区域，返回0
}

double RRT3D::compute_gain(Node *q_new)
{
    double gain = 0;
    if(q_new->parent==NULL){
        return gain;
    }    

    cout << "q_new->parent: " << q_new->parent->position.x() << " " << q_new->parent->position.y()<< " " << q_new->parent->position.z() << endl;
    
    // gain分为与目标点的距离+与边界的距离

    double gain_border_distance = border_gain(q_new);
    ROS_INFO("gain_border_distance: %f", gain_border_distance);

    double parent_distance;
    parent_distance = (q_new->parent->position - end_position_).norm();

    double current_distance;
    current_distance = (q_new->position - end_position_).norm();

    double gain_goal_distance = current_distance - parent_distance;
    ROS_INFO("gain_goal_distance: %f", gain_goal_distance);


    // gain = q_new->parent->gain - 10*gain_goal_distance - gain_border_distance;
    
    if(q_new->parent!=NULL){
        gain = 5*gain_goal_distance - gain_border_distance;
        ROS_INFO("node_gain: %f", gain);
    }
    else{
        gain = 5*gain_goal_distance - gain_border_distance;
        ROS_INFO("root_node_gain: %f", gain);
    }
    
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

    if((lastNode_->position - end_position_).norm() < 2*map_->getResolution())
        return true;
    return false;
}

bool RRT3D::run()
{
    srand(static_cast<ushort>(time(NULL)));
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for(int i=0; i<max_iter_; i++){

        std::cout<<"i="<<i<<std::endl;
        Node *q_rand = getRandomNotObstacleNode();
        ROS_INFO("q_rand: %f %f %f", q_rand->position.x(), q_rand->position.y(), q_rand->position.z());
        Node *q_nearest = findNearestNode(q_rand->position);

//        std::cout<<"nearest_node: "<<q_nearest->position<<std::endl;
        octomap::point3d direction = q_rand->position - q_nearest->position;
        // ROS_INFO("direction: %f %f %f", direction.x(), direction.y(), direction.z());
        // std::cout<<"enter run 3" <<std::endl;
        // Node *node_new;
        if(direction.norm() > step_size_){
            direction = direction.normalize();
            Node *q_new = getNewNode(q_rand, q_nearest, direction);
            if(!isNewNodeCollision(q_new->position, q_nearest->position, direction)){
                addNewNode(q_nearest, q_new);
            }
            
            // node_new = q_new;
        }
        else{
            // addNewNode(q_nearest, q_rand);
            // double gain_tmp = compute_gain(q_rand);
            // node_new = q_rand;
            addNewNode(q_nearest, q_rand);
        }
        
        // ROS_INFO("node_new: %f %f %f", node_new->position.x(), node_new->position.y(), node_new->position.z());
        // addNewNode(q_nearest, node_new);
        // node_new->gain = compute_gain(node_new);

        // if(node_new->gain < 0){
        //     node_new->gain = node_new->parent->gain + node_new->gain;
        //     ROS_INFO("Total_Gain_: %f", node_new->gain);

        //     if (node_new->gain < bestGain_) {
        //         ROS_INFO("bestGain_: %f", bestGain_);
        //         bestGain_ = node_new->gain;
        //         bestNode_ = node_new;
        //     }
        // }
        // else{
        //     // delete
        //     deleteNode(node_new);
        // }
        
        // if(isArrived()){
        //     ROS_ERROR("lastNode_: %f %f %f", lastNode_->position.x(), lastNode_->position.y(), lastNode_->position.z());
        //     ROS_ERROR("Find a path......");
        //     break;
        // }
        if(isArrived()){
            break;
        }
    }

    if(isArrived()){
        Node *q;
        // q = bestNode_;
        q = lastNode_;

        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
        while(q!=NULL){
            path_.push_back(q);
            q = q->parent;
        }

        return true;
    }
    else{
        Node *q;
        // q = bestNode_;
        // q=lastNode_;
        q = findNearestNode(end_position_);

        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
        while(q!=NULL){
            path_.push_back(q);
            q = q->parent;
        }

        return true;
    }
    

    // Node *q;
    // if(isArrived()){
    //     q = lastNode_;
    //     std::cout<<"Find a path\n";
    // }
    // else{
    //     q = findNearestNode(end_position_);
    //     std::cout<<"Can not find the path\n";
    // }
    // end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end-start;
    // std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
    // while(q!=NULL){
    //     path_.push_back(q);
    //     q = q->parent;
    // }

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

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT3D::deleteAllNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}

void RRT3D::deleteNode(Node *node)
{
    octomap::point3d position_tmp = node->position;
    Node *parent_node = node->parent;
    ROS_WARN("Children.size: %d", parent_node->children.size());

    if(parent_node->children.size() == 1){
        ROS_WARN("delete children: %f %f %f", node->position.x(),node->position.y(),node->position.z());
        parent_node->children.clear();
        ROS_WARN("Children.size: %d", parent_node->children.size());
    }
    else{
        vector<Node *>::iterator it;

        for (it = parent_node->children.begin(); it != parent_node->children.end(); ++it){
            ROS_WARN("delete children: %f %f %f", (*it)->position.x(),(*it)->position.y(),(*it)->position.z());
            if(abs(position_tmp.x() - (*it)->position.x())<0.0005 && abs(position_tmp.y() - (*it)->position.y())<0.0005 && abs(position_tmp.z() - (*it)->position.z())<0.0005){
                ROS_WARN("delete children: %f %f %f", (*it)->position.x(),(*it)->position.y(),(*it)->position.z());
                parent_node->children.erase(it);
                ROS_WARN("Children.size: %d", parent_node->children.size());
                break;
            }    
        }

        for(it=parent_node->children.begin();it!=parent_node->children.end();it++)
        {
            ROS_WARN("delete children: %f %f %f", (*it)->position.x(),(*it)->position.y(),(*it)->position.z());
        }	
    }
    
}