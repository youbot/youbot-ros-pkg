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

//BRICS_3D specific headers
#include "examples/EuclideanClusterExtractor.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>

/**
 * usage: rosrun perception_sdk_pkg objectClusterExtractor <no_of_regions> <max_no_of_objects_in_each_region>
 *
 * Finds object clusters from the hsv-based extracted regions of interests
 *
 * Arguments:
 * <no_of_regions> 	: corresponds to the count of difrent regions of intersets, ex: green, red
 * 					: default value 1
 * <max_no_objects> : max number of object clusters possible in each region
 * 					: default value 1
 * Topic published:
 *  region_1_obj_cluster_1, region_1_obj_cluster_2, region_2_obj_cluster_1 ......
 *  type: Pointcloud2 message
 *  frame_id: openni_rgb_optical_frame
 */


//global variables
BRICS_3D::EuclideanClusterExtractor *objectClusterExtractor;
int maxNoOfObjects;
int noOfRegions;

//void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
//	for(int i=0; i<noOfRegions;i++){
//		ROS_INFO("received a kinect message...");
//		objectClusterExtractor[i].kinectCloudCallback(cloud);
//	}
//}

int main(int argc, char* argv[]){

	ros::init(argc, argv, "ObjectClusterExtractor");
	ros::NodeHandle nh;

	if(argc == 2){
		noOfRegions = atoi(argv[1]);
		maxNoOfObjects = 1;
	} else if(argc == 3){
		noOfRegions = atoi(argv[1]);
		maxNoOfObjects = atoi(argv[2]);
	} else {
		ROS_INFO("Using default values");
		noOfRegions = 1;
		maxNoOfObjects = 1;
	}

	ROS_INFO("Looking for at most [%d] object-cluster(s) in [%d] extracted regions",
																maxNoOfObjects, noOfRegions);

	//Define the publishers for each extracted region
	ros::Publisher extractedClusterPublisher[noOfRegions][maxNoOfObjects];

	//Define the cluster extractors
	//Todo replace malloc
	if( (objectClusterExtractor = (BRICS_3D::EuclideanClusterExtractor *)malloc
			(noOfRegions*sizeof(BRICS_3D::EuclideanClusterExtractor ))) == NULL ){
		ROS_ERROR("Memory Allocation Error!!");
		exit(0);
	}


	for (int i = 0; i<noOfRegions; i++){
		//initialize the publishers
		for(int j=0; j<maxNoOfObjects; j++){
			std::stringstream pubTopic;
			pubTopic.str("");
			pubTopic.clear();
			pubTopic << "region_" << i+1 << "_obj_cluster_" << j+1;
			extractedClusterPublisher[i][j] = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >
																				(pubTopic.str(), 1);
		}

		//initialize the object cluster extractor
		objectClusterExtractor[i].initializeExtractor(maxNoOfObjects,extractedClusterPublisher[i],
																				200,25000, 0.01);
	}

	//subscribe to kinect point cloud messages
	//extracted_region_

    ros::Subscriber  extractedRoiSubscriber[noOfRegions];

    for (int i = 0; i < noOfRegions ; i++){
    	std::stringstream subTopic;
    	subTopic.str("");
    	subTopic.clear();
    	subTopic << "extracted_region_"<< i+1;
    	extractedRoiSubscriber[i]= nh.subscribe(subTopic.str(), 1,
    			&BRICS_3D::EuclideanClusterExtractor::kinectCloudCallback, &objectClusterExtractor[i]);
    }

    ROS_INFO("Now extracting Object Clusters ;)");

	ros::spin();

	free(objectClusterExtractor);
	return 0;
}
