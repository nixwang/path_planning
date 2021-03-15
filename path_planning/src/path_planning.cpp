 /*
 * Copyright 2017 Ayush Gaud 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string.h>
#include <octomap/ColorOcTree.h>


#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <iostream>

#include "../include/octomap_rrt.hpp"
#include "../include/map.hpp"
#include "../include/montecarlo.hpp"

using namespace std;
using namespace octomap;

// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher pub_octomap;

nav_msgs::Path path;

point3d start_position(0,0.3,0);
point3d cur_position(0,0.3,0);
point3d end_position(10,0.3,0);
double init_gain = 0;
int max_iterator_path_planner = 60000;
int max_iterator_view_planner = 200;
int step_size_path_planner = 2;
double search_radius = 5;	// 视角规划中的搜索半径
bool rrt_done = true;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
	// octomap::ColorOcTree* cot = octomap_msgs::msgToMap(msg);

	std::cout << "entered octomapCallback..." << std::endl;

	// rrt_done = false;
	bool color_ = false;
	Map* map = new Map(*msg, color_);

	/******************* 路径规划部分： ******************/
	RRT3D rrt_path_planer(start_position, end_position, map, max_iterator_path_planner, step_size_path_planner);
	rrt_done = rrt_path_planer.run();

	if(rrt_done){
		// 显示轨迹
		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();

		path.header.stamp = current_time;
		path.header.frame_id = "world";

		path.poses.clear();
		for(auto node:rrt_path_planer.path_){
			// std::cout<<node->position<<std::endl;
			// map_->mixPathMap(node->position, true);

			geometry_msgs::PoseStamped this_pose_stamped;
			geometry_msgs::PointStamped this_point_stamped;

			this_pose_stamped.pose.position.x = node->position.x();
			this_pose_stamped.pose.position.y = node->position.y();
			this_pose_stamped.pose.position.z = node->position.z();

			this_point_stamped.header.stamp = current_time;
			this_point_stamped.header.frame_id = "world";
			this_point_stamped.point.x = node->position.x();
			this_point_stamped.point.y = node->position.y();
			this_point_stamped.point.z = node->position.z();
			ROS_INFO("current_pos: %f %f %f", node->position.x(), node->position.y(), node->position.z());
			// ROS_INFO("current_y: %f", node->position.y());
			// ROS_INFO("current_z: %f", node->position.z());
			geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
			this_pose_stamped.pose.orientation.x = goal_quat.x;
			this_pose_stamped.pose.orientation.y = goal_quat.y;
			this_pose_stamped.pose.orientation.z = goal_quat.z;
			this_pose_stamped.pose.orientation.w = goal_quat.w;

			this_pose_stamped.header.stamp = current_time;
			this_pose_stamped.header.frame_id = "world";
			path.poses.push_back(this_pose_stamped);
		}
	}
	
	// 显示路径规划的八叉树
	// octomap_msgs::Octomap map_msg;
	// map_msg.header.frame_id = "world";
	// map_msg.header.stamp = ros::Time::now();
	// if (octomap_msgs::fullMapToMsg(*(map->map_tree_), map_msg))
	// 	pub_octomap.publish(map_msg);
	// else
	// 	ROS_ERROR("Error serializing OctoMap");

	cout << "rrt_done "  << rrt_done << endl;
	// rrt_path_planer.writeMap();
	// // map->writeFile("./path_octotree.bt");

	// rrt_path_planer.writeInfo2File("/home/hitwzh/semantic_slam_ws/src/path_planning/path/path_distance.txt");

	/**************** 轨迹规划部分：*****************/
	// 结合两栖机器人的离散步态（前进、后退、左转、右转）与路径规划的结果（rrt采样得到的路径点）进行优化，得到一系列可行的轨迹点与机器人的运动序列（如，前进+前进+左转+左转+前进）
	// 即，在RRT规划结果中的下一个路径点周围区域内，寻找下一个可行轨迹点。具体的可自由发挥。


	/**************** 视角规划部分：*****************/
	// 通过采样得到最优视角的落点，采样均限制在一个圆形区域内（受云台运动角度范围限制），
	// 并且考虑相机不同视角落点对应不同的区域
	view_planner view_planner_(cur_position, map, max_iterator_view_planner, search_radius);

	Position best_view_point = view_planner_.run_ViewPlanner();

	
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	// cout << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " <<  msg->pose.pose.position.z << endl;
	// octomap::point3d odom_cb(msg->pose.pose.position.x - 3.5, msg->pose.pose.position.z, msg->pose.pose.position.y - 2);
	// start_position = odom_cb;	
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	// planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
	// planner_ptr->init_start();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	// octomap::point3d goal_cb(msg->point.x - 3.5, msg->point.y - 2, msg->point.z + 0.016);
	// end_position = goal_cb;	
	// planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	// planner planner_object;

	// Map* map = new Map("./test_map_complex.bt");

	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_full_semantic", 1, octomapCallback);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/ground_truth/feedback", 1, odomCb);
	// ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, goalCb);
	// ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
	pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap_full_second", 1, true);

	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory", 1, true);
	
	ros::Rate loop_rate(10);


    while (ros::ok())
    {
		// ROS_INFO("This is an info statement, and should print");

		if(rrt_done){
			path_pub.publish(path);
		}        

		ros::spinOnce();

		loop_rate.sleep();
    }
	// std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	// ros::spin();

	return 0;
}
