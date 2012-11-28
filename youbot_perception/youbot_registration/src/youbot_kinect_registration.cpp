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
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <cv.h>
#include <highgui.h>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#define NB_ARM_JOINTS 5

ros::Publisher worldpts_pub;
ros::Publisher targetpts_pub;
ros::Publisher armPositionsPublisher;
ros::Subscriber image_sub;
ros::Subscriber pcl_sub;
ros::ServiceClient registration_client;
tf::TransformListener *tf_listener;
ros::Timer timer_move;
ros::Timer timer_send;

cv::Point pixelPoint;
pcl::PointCloud<pcl::PointXYZ> cloud;
cv::Mat image;

bool firstrun = true;

int number_of_points;

const double starting_jointPosition[NB_ARM_JOINTS] = {2.56244, 1.04883, -2.43523, 1.73184, 0.2};

const double predefined_jointPositions[][NB_ARM_JOINTS] =  {
                                                {0.1,0.1,-2.5,2.5, 2.2}
                                                ,{0.1,0.1,-2.5,2.5, 3.0}
                                                ,{0.1,0.1,-3.14,3.14, 3.0}
                                                ,{0.1,0.1,-3.14,3.14, 2.2}
                                                ,{0.5,0.1,-2.5,2.5, 2.2}
                                                ,{0.5,0.1,-2.5,2.5, 3.0}
                                                ,{0.5,0.1,-3.14,3.14, 3.0}
                                                ,{0.5,0.1,-3.14,3.14, 2.2}
                                                ,{0.9,0.1,-2.5,2.5, 2}
                                                ,{0.9,0.1,-2.5,2.5, 2.5}
                                                ,{0.9,0.1,-3.14,3.14, 2.5}
                                                ,{0.9,0.1,-3.14,3.14, 2}
                                                ,{1.3,0.1,-2.5,2.5, 1.57}
                                                ,{1.3,0.1,-2.5,2.5, 2.5}
                                                ,{1.3,0.1,-3.14,3.14, 2.5}
                                                ,{1.3,0.1,-3.14,3.14, 1.57}
//                                                ,{1.7,0.1,-2.5,2.5, 1.57}
//                                                ,{1.7,0.1,-2.5,2.5, 2.5}
//                                                ,{1.7,0.1,-3.14,3.14, 2.5}
//                                                ,{1.7,0.1,-3.14,3.14, 1.57}
//                                                ,{2.1,0.1,-2.5,2.5, 1.0}
//                                                ,{2.1,0.1,-2.5,2.5, 1.57}
//                                                ,{2.1,0.1,-2.5,2.5, 2.5}
//                                                ,{2.1,0.1,-3.14,3.14, 2.5}
//                                                ,{2.1,0.1,-3.14,3.14, 1.57}
//                                                ,{2.1,0.1,-3.14,3.14, 1.0}
//                                                ,{2.5,0.1,-2.5,2.5, 1.57}
//                                                ,{2.5,0.1,-2.5,2.5, 2.5}
//                                                ,{2.5,0.1,-3.14,3.14, 2.5}
//                                                ,{2.5,0.1,-3.14,3.14, 1.57}
                                                ,{2.56244, 1.04883, -2.43523, 1.73184, 0.2}
//                                                ,{4.6,0.1,-2.5,2.5,4.2}
//                                                ,{4.6,0.1,-2.5,2.5,3.5}
//                                                ,{4.6,0.1,-3.14,3.14,3.5}
//                                                ,{4.6,0.1,-3.14,3.14,4.2}
//                                                ,{5.0,0.1,-2.5,2.5,4}
//                                                ,{5.0,0.1,-2.5,2.5,3.14}
//                                                ,{5.0,0.1,-3.14,3.14,3.14}
//												,{5.0,0.1,-3.14,3.14,4}
												,{5.4,0.1,-2.5,2.5,4}
												,{5.4,0.1,-2.5,2.5,3.14}
												,{5.4,0.1,-3.14,3.14,3.14}
												,{5.4,0.1,-3.14,3.14,4}
												,{5.8,0.1,-2.5,2.5,4}
												,{5.8,0.1,-2.5,2.5,3.14}
												,{5.8,0.1,-3.14,3.14,3.14}
												,{5.8,0.1,-3.14,3.14,4}
};

bool detectCirclesGrid()
{
	if (image.size().width)
	{
		cv::Size patternsize(7,7); //number of centers
		cv::Mat imgresize;
		cv::resize(image,imgresize,cv::Size(image.size().width*2,image.size().height*2));
		cv::Mat gray(imgresize.size(), CV_8U);
		cv::cvtColor(imgresize, gray, CV_BGRA2GRAY);
		std::vector<cv::Point2f> centers; //this will be filled by the detected centers

		bool patternfound =  false;
		for (int i =0; i<10;i++)
		{
			if (findCirclesGrid(gray, patternsize, centers))
			{
				patternfound = true;
				break;
			}
		}
		if (patternfound)
		{
			pixelPoint.x = centers[24].x/2; // Divided by 2 because the image is enlarged
			pixelPoint.y = centers[24].y/2;
			return true;
		}
		ROS_WARN("FindCircles failed");
	}
	else
	{
		ROS_WARN("Image is empty");
	}
    return false;
}

void imageReceivedCallback(const sensor_msgs::Image::ConstPtr &img)
{
	cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(img);
	cvtColor(ptr->image.clone(), image, CV_BGR2BGRA);

	if (firstrun)
	{
		firstrun = false;
		timer_move.start();
	}
}

void pclReceivedCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_pcl)
{
	pcl::fromROSMsg(*ros_pcl,cloud);
}

bool send_target_point()
{
	pcl::PointXYZ point = cloud(pixelPoint.x,pixelPoint.y);

	    if (!isnan(point.z) && !isnan(point.y) && !isnan(point.x))
	    {
	        geometry_msgs::PointStamped tempPoint;
	        geometry_msgs::PointStamped outPoint;
	        tempPoint.point.x = point.x;
	        tempPoint.point.y = point.y;
	        tempPoint.point.z = point.z;
	        tempPoint.header.frame_id = cloud.header.frame_id;

	        tf_listener->transformPoint("openni_camera",tempPoint,outPoint);

	        targetpts_pub.publish(outPoint);
	        return true;
	    }
	    ROS_WARN("Range is too near or too far");
	    return false;
}

void send_world_point()
{
	tf::StampedTransform transform;
	tf_listener->lookupTransform("base_footprint","pattern_center_circle",
			ros::Time(0),transform);

	geometry_msgs::PointStamped world_point;
	world_point.point.x = transform.getOrigin().x();
	world_point.point.y = transform.getOrigin().y();
	world_point.point.z = transform.getOrigin().z();
	world_point.header.frame_id = "base_footprint";

	worldpts_pub.publish(world_point);
}

void timerMoveCallback(const ros::TimerEvent event)
{
	if (number_of_points != 0)
	{
		timer_move.stop();
		brics_actuator::JointPositions command;
		std::vector <brics_actuator::JointValue> armJointPositions;
		armJointPositions.resize(NB_ARM_JOINTS);

		for (int j = 0; j < NB_ARM_JOINTS; j++)
		{
			std::stringstream jointName;
			jointName << "arm_joint_" << (j + 1);
			armJointPositions[j].joint_uri = jointName.str();
			armJointPositions[j].value = predefined_jointPositions[number_of_points-1][j];
			armJointPositions[j].unit = boost::units::to_string(boost::units::si::radians);
		}

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);
		ROS_INFO("Joint Published");

		ros::Rate rate(1);

		for (int i = 0; i < 2; i++)
		{
			rate.sleep();
		}
	}
	timer_send.start();
}

void timerSendCallback(const ros::TimerEvent &event)
{
	if (number_of_points != 0)
	{
		if (detectCirclesGrid())
		{
			if (send_target_point())
			{
				send_world_point();
				ROS_INFO("Point sent");
			}
		}
		number_of_points--;
		timer_move.start();
	}
	else
	{
		std_srvs::Empty srv;

		if (registration_client.call(srv))
		{
			ROS_INFO("Registration Succeed");
		}
		else
		{
			ROS_ERROR("Registration Failed");
		}
		timer_move.stop();
	}
	timer_send.stop();
}

void timerStartCallback(const ros::TimerEvent &event)
{
	brics_actuator::JointPositions command;
	std::vector <brics_actuator::JointValue> armJointPositions;
	armJointPositions.resize(NB_ARM_JOINTS);

	for (int j = 0; j < NB_ARM_JOINTS; j++)
	{
		std::stringstream jointName;
		jointName << "arm_joint_" << (j + 1);
		armJointPositions[j].joint_uri = jointName.str();
		armJointPositions[j].value = starting_jointPosition[j];
		armJointPositions[j].unit = boost::units::to_string(boost::units::si::radians);
	}

	command.positions = armJointPositions;
	armPositionsPublisher.publish(command);
	ROS_INFO("Starting Joint Published");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "youbot_kinect_registration");
	ros::NodeHandle n;

	worldpts_pub = n.advertise<geometry_msgs::PointStamped>("youbot/sensor/registration/world", 1);
	targetpts_pub =	n.advertise<geometry_msgs::PointStamped>("youbot/sensor/registration/target", 1);
	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);

	image_sub = n.subscribe("camera/rgb/image_color",1,&imageReceivedCallback);
	pcl_sub = n.subscribe("camera/rgb/points",1,&pclReceivedCallback);

	registration_client = n.serviceClient<std_srvs::Empty>("youbot/sensor/registration/computeMatrix");

    tf_listener = new tf::TransformListener(n);

    ros::Timer timer_start = n.createTimer(ros::Duration(1.0),&timerStartCallback,true,true);
    timer_move = n.createTimer(ros::Duration(1.0),&timerMoveCallback,false,false);
    timer_send = n.createTimer(ros::Duration(1.0),&timerSendCallback,false,false);

    number_of_points = sizeof( predefined_jointPositions ) / (sizeof( predefined_jointPositions[0][0]) * NB_ARM_JOINTS);

    ros::spin();

	return 0;
}
