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

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"


#include "../include/octomap_rrt.hpp"
#include "../include/map.hpp"
using namespace std;
using namespace octomap;

// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

point3d start_position(0,0,0);
point3d end_position(2,2,0);
double init_gain = 0;
int max_iterator_ = 1;
int step_size_global = 1;
bool rrt_done = true;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
	// octomap::ColorOcTree* cot = octomap_msgs::msgToMap(msg);

	if(rrt_done){
		std::cout << "entered octomapCallback..." << std::endl;

		rrt_done = false;
		bool color_ = false;
		Map* map = new Map(*msg, color_);
		// cout << map->getBBXMax().x() << endl;
		// cout << map->getResolution() << endl;

		// octomap::point3d start_position(0,0,0);
		// octomap::point3d end_position(39,20,9);

		cout << start_position.x() << " " << start_position.y() << " " <<  start_position.z() << endl;
		// octomap::point3d end_position(39,20,9);
		RRT3D rrt(start_position, end_position, map, max_iterator_, step_size_global);
		rrt_done = rrt.run(false);
		// rrt.writeMap();
		// // map->writeFile("./path_octotree.bt");

		// std::cout << "rrt running..." << std::endl;

		// rrt.writeInfo2File("/home/hitwzh/semantic_slam_ws/src/path_planning/path/path_distance.txt");
	}
	
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	// cout << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " <<  msg->pose.pose.position.z << endl;
	// octomap::point3d odom_cb(msg->pose.pose.position.x - 3.5, msg->pose.pose.position.y - 2, msg->pose.pose.position.z + 0.016);
	// start_position = odom_cb;	
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	// planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
	// planner_ptr->init_start();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	octomap::point3d goal_cb(msg->point.x - 3.5, msg->point.y - 2, msg->point.z + 0.016);
	end_position = goal_cb;	
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
	ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, goalCb);
	// ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
	
	// std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}
