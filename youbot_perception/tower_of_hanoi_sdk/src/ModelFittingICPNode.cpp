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
#include "examples/ModelFitting.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>



int main(int argc, char* argv[]){


	int maxNoOfObjects;
	int noOfRegions;

	ros::init(argc, argv, "ModelFittingICP");
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

	ROS_INFO("Finding pose for at most [%d] object-cluster(s) in [%d] extracted regions",
																maxNoOfObjects, noOfRegions);

	//Define the publishers for each estimated model
	ros::Publisher estimatedModelPublisher[noOfRegions][maxNoOfObjects];

	//Define the model estimators
	brics_3d::ModelFitting poseEstimator[noOfRegions][maxNoOfObjects];


	for (int i = 0; i<noOfRegions; i++){
		//initialize the publishers
		for(int j=0; j<maxNoOfObjects; j++){
			std::stringstream pubTopic;
			pubTopic.str("");
			pubTopic.clear();
			pubTopic << "region_" << i+1 << "_obj_model_" << j+1;
			estimatedModelPublisher[i][j] = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >
																				(pubTopic.str(), 1);
		}
	}

	//subscribe to kinect point cloud messages
	ros::Subscriber  objectClusterSubscriber[noOfRegions*maxNoOfObjects];
	int count = 0;
    for (int i = 0; i < noOfRegions ; i++){
    	for (int j = 0; j < maxNoOfObjects; j++) {
        	std::stringstream subTopic;
        	subTopic.str("");
        	subTopic.clear();
        	subTopic << "region_"<< i+1 << "_obj_cluster_" << j+1;
        	objectClusterSubscriber[count]= nh.subscribe(subTopic.str(), 1,
        			&brics_3d::ModelFitting::kinectCloudCallback, &poseEstimator[i][j]);
        	poseEstimator[i][j].setModelPublisher(&estimatedModelPublisher[i][j]);
        	count++;
		}
    }

    ROS_INFO("Now estimating models;)");

	ros::spin();

	//free(objectClusterExtractor);
	return 0;
}
