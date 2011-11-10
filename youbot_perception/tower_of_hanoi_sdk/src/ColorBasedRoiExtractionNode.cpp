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

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"

//BRICS_3D specific Headers
#include "examples/ColorBasedRoiExtractor.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>


using namespace std;
BRICS_3D::ColorBasedRoiExtractor *roiExtractor;
int noOfRegions = 0;


void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	for(int i=0; i<noOfRegions;i++){
		ROS_INFO("received a kinect message...");
		roiExtractor[i].kinectCloudCallback(cloud);
	}
}

int main(int argc, char* argv[]){

	ros::init(argc, argv, "ColorBasedRoiExtractor");
	ros::NodeHandle nh;

	if(argc < 2){
		ROS_ERROR("Not enough arguments:\n Usage:\t "
				"rosrun colorBasedRoiExtractor <no_of_regions> <region_1_config_file> "
				"<region_2_config_file>....");
		exit(0);
	}


	//Set up no of regions requested by the user for extraction
	noOfRegions = atoi(argv[1]);

//    noOfRegions = 1;
//    argv[2] = "/home/pinaki/hack-arena/ROS/tower_of_hanoi_sdk/redRoiConfig.cfg";

	//Define the publishers for each extracted region
	ros::Publisher extractedRoiPublisher[noOfRegions];

	//Define the color based region extractors

	if( (roiExtractor = (BRICS_3D::ColorBasedRoiExtractor *)malloc(noOfRegions*sizeof(BRICS_3D::ColorBasedRoiExtractor))) == NULL ){
		ROS_ERROR("Memory Allocation Error!!");
		exit(0);
	}

	//define the HSV limit variables;
	float minLimitH[noOfRegions], minLimitS[noOfRegions],
			maxLimitH[noOfRegions], maxLimitS[noOfRegions];

	//parse the configuration files and set up the HSV limits
	ifstream configFileStream;


	for (int i = 0; i<noOfRegions; i++) {
		configFileStream.open(argv[i+2]);
		      if ( configFileStream.is_open() ) {     //if file exists
		          string s;
		          while(getline(configFileStream, s)){    //extract the values of the parameters
		              	std::vector< std::string > tempVec;
		                  boost::split(tempVec, s, boost::is_any_of("="));
		                  if(!tempVec[0].compare("minH")) {
		                       minLimitH[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("maxH")) {
		                       maxLimitH[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("minS")) {
		                       minLimitS[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("maxS")) {
		                       maxLimitS[i] = atof(tempVec[1].c_str());
		                  }
		          }
		          configFileStream.close();
		      } else {
		    	  ROS_ERROR("Configuration file: %s not found!!", argv[i+2]);
		    	  exit(0);
		      }
	}



	//initialize each color based region extractor
	for (int i = 0; i<noOfRegions; i++) {
		roiExtractor[i].initializeLimits(minLimitH[i], maxLimitH[i], minLimitS[i], maxLimitS[i]);

		stringstream pubMessageName;
		pubMessageName << "extracted_region_"<<i+1;
		extractedRoiPublisher[i] = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  (pubMessageName.str(), 1);
		roiExtractor[i].setExtractedRegionPublisher(&extractedRoiPublisher[i]);
	}




	//subscribe to kinect point cloud messages
    ros::Subscriber  kinectCloudSubscriber = nh.subscribe("/camera/rgb/points", 1,&kinectCloudCallback);

    ROS_INFO("Now extracting ROIs ;)");

	ros::spin();

	free(roiExtractor);
	return 0;
}
