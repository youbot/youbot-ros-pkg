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
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <cv.h>
#include <highgui.h>

ros::Subscriber image_sub;

cv::Point pixelPoint;
cv::Mat image;

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


	if (detectCirclesGrid())
	{
		ROS_INFO("Pattern found.");
	} else {
		ROS_ERROR("Pattern not found");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pattern_detection_test");
	ros::NodeHandle n;

	image_sub = n.subscribe("camera/rgb/image_color",1,&imageReceivedCallback);

    ros::spin();

	return 0;
}
