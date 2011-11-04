/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

//ROS specific Headers
#include <ros/ros.h>
#include "tower_of_hanoi_sdk/Configuration.h"
#include "ros/publisher.h"
#include <iostream>
#include <stdio.h>
int main(int argc, char* argv[]){

	ros::init(argc, argv, "ConfigurePerception");
	ros::NodeHandle nh;

	ros::Publisher perceptionCommandPublisher = nh.advertise<tower_of_hanoi_sdk::Configuration>  ("/perceptionConfiguration", 1);


	std::string choice;
	bool configure = true;
	tower_of_hanoi_sdk::Configuration message;

	while(configure){

		std::cout << "\n Please enter the maximum number of regions : \n ";
		std::getline(std::cin, choice);
		message.no_of_regions = atoi(choice.c_str());

		std::cout << " Please enter the maximum number of objects to be searched for in each region"
				" : \n";
		std::getline(std::cin, choice);
		message.max_no_of_objects = atoi(choice.c_str());

		std::cout << " Enter the  HSV-Limits configuration filenames for all the regions  "
				"(separated by space) : \n";
		std::getline(std::cin, choice);
		message.config_files = choice;

		std::cout << " Enter the  labels for each region to be used for broadcasting the transforms"
				" for the objects (separated by space) : \n";
		std::getline(std::cin, choice);
		message.labels = choice;

		std::cout << " Send Configuration Settings? (y/n)  : ";
		std::getline(std::cin, choice);
		if(!choice.compare("y")){
			perceptionCommandPublisher.publish(message);
		}


		configure = false;
		std::cout << " Configure Again? (y/n)  : ";
		std::getline(std::cin, choice);
		if(!choice.compare("y")){
			configure = true;
		}
	}

}
