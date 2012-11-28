/******************************************************************************
* Copyright (c) 2012
* Locomotec
*
* Author:
* Constantinos Lantos (inos Automationssoftware GmbH)
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/src/Core/Matrix.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

pcl::PointCloud<pcl::PointXYZ> world;
pcl::PointCloud<pcl::PointXYZ> target;
std::string world_name;
std::string target_name;
tf::Transform transform;
bool broadcast = false;

void worldReceivedCallback(const geometry_msgs::PointStamped::ConstPtr & wPoint)
{
	world.push_back(pcl::PointXYZ(wPoint->point.x, wPoint->point.y, wPoint->point.z));
	world_name = wPoint->header.frame_id;
}

void targetReceivedCallback(const geometry_msgs::PointStamped::ConstPtr & tPoint)
{
	target.push_back(pcl::PointXYZ(tPoint->point.x, tPoint->point.y, tPoint->point.z));
	target_name = tPoint->header.frame_id;
}

void broadcastTF(const ros::TimerEvent& event)
{
	if (broadcast)
	{
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
											  world_name, target_name));
	}
}

bool computeMatrix(std_srvs::Empty::Request &req,
				   std_srvs::Empty::Response &res)
{
	if ((!world_name.empty()) && (!target_name.empty()) &&
		(target.size() > 2) && (world.size() == target.size()))
	{
		Eigen::Matrix4f trMatrix;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
		svd.estimateRigidTransformation(target, world, trMatrix);

		ROS_INFO("Registration completed and Registration Matrix is being broadcasted");

		transform = tf::Transform(btMatrix3x3(trMatrix(0, 0), trMatrix(0, 1), trMatrix(0, 2),
											  trMatrix(1, 0), trMatrix(1, 1), trMatrix(1, 2),
											  trMatrix(2, 0), trMatrix(2, 1), trMatrix(2, 2)),
								  btVector3(trMatrix(0, 3), trMatrix(1, 3), trMatrix(2, 3)));

		broadcast = true;
		return true;
	}
	else
	{
		return false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "youbot_sensor_registration");
	ros::NodeHandle n;

	ros::Subscriber worldpts_sub = n.subscribe("youbot/sensor/registration/world", 1, worldReceivedCallback);
	ros::Subscriber targetpts_sub =	n.subscribe("youbot/sensor/registration/target", 1,targetReceivedCallback);

	ros::ServiceServer service = n.advertiseService("youbot/sensor/registration/computeMatrix",computeMatrix);

	ros::Timer timer = n.createTimer(ros::Duration(0.01), broadcastTF);

	ros::spin();
}
