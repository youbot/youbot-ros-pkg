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
#include "sensor_msgs/PointCloud2.h"
#include "tower_of_hanoi_sdk/Coordination.h"
#include "tower_of_hanoi_sdk/Configuration.h"
#include "ros/publisher.h"

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"

//BRICS_3D specific headers
#include "examples/PoseEstimation6D.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>



//global variables
std::vector<BRICS_3D::PoseEstimation6D*> poseEstimators;
std::vector<BRICS_3D::PointCloud3D*> modelDataBase;
std::vector<std::string> modelNames;
bool perceptionPaused;
unsigned int maxNoOfObjects;
unsigned int noOfRegions;


void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	if(!perceptionPaused){
		for(unsigned int i=0; i<noOfRegions;i++){
			//			ROS_INFO("received a kinect message...");
			poseEstimators[i]->kinectCloudCallback(cloud);
		}
	} else {
//		ROS_INFO("Perception Engine Paused");
	}
}


void perceptionControlCallback(const tower_of_hanoi_sdk::Coordination message){



	if(!message.command.compare("pause")){
		perceptionPaused=true;
		ROS_WARN("Pausing Perception Engine");
	}else if(!message.command.compare("resume")){
		perceptionPaused=false;
		ROS_WARN("Resuming Perception Engine");
	}

}


void perceptionConfigurationCallback(const tower_of_hanoi_sdk::Configuration message){

	//Checking if the new configuration is valid or not
	std::vector< std::string > configFiles;
	boost::split(configFiles, message.config_files, boost::is_any_of(" "));
//	ROS_WARN("[CHEAT] %s", message.labels.c_str() );
	std::vector< std::string > labels;
	boost::split(labels, message.labels, boost::is_any_of(" "));

	std::ifstream configFileStream;

	unsigned int noOfRegions_ = message.no_of_regions;


	if(configFiles.size()!=noOfRegions_ ){
			ROS_WARN("Invalid configuration setting received: not enough config files");
//			ROS_WARN("[CHEAT] %d %d", configFiles.size(), noOfRegions_ );
			return;
	}

	if(labels.size()!=noOfRegions_ ){
		ROS_WARN("Invalid configuration setting received: not enough labels");
//		ROS_WARN("[CHEAT] %d %d", labels.size(), noOfRegions_ );
		return;
	}
	for(unsigned int i = 0; i<noOfRegions_; i++){
		configFileStream.open(configFiles[i].c_str());
		if ( configFileStream.is_open() ) {     //if file exists
			configFileStream.close();
		} else {
			ROS_WARN("Configuration file: '%s' not found!!", configFiles[i].c_str());
			ROS_WARN("Invalid configuration setting received");
			return;
		}
	}
	//Setting up new configuration
	perceptionPaused=true;
	ROS_WARN("Pausing Perception Engine");

	maxNoOfObjects = message.max_no_of_objects;
	noOfRegions = noOfRegions_;

	if(poseEstimators.size() < noOfRegions){
		for(unsigned int i=0; i< noOfRegions-poseEstimators.size(); i++){
			poseEstimators.push_back(new BRICS_3D::PoseEstimation6D());
		}
	}

	//define the HSV limit variables;
	float minLimitH[noOfRegions], minLimitS[noOfRegions],
	maxLimitH[noOfRegions], maxLimitS[noOfRegions];

	for(unsigned int i = 0; i<noOfRegions; i++){
		configFileStream.open(configFiles[i].c_str());
		if ( configFileStream.is_open() ) {     //if file exists
			std::string s;
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
			ROS_ERROR("Configuration file: %s not found!!", configFiles[i].c_str());
			exit(0);
		}

		//Setting the max-no of objects to be searched for
		poseEstimators[i]->setMaxNoOfObjects(maxNoOfObjects);
		//Setting the label which will be used to publish the transforms
		poseEstimators[i]->setRegionLabel(labels[i]);
		//Initializing the limits in HSV space to extract the ROI
		poseEstimators[i]->initializeLimits(minLimitH[i], maxLimitH[i], minLimitS[i], maxLimitS[i]);
		//Initializing the cluster extractor limits
		//ToDo Allow configurating these parameters too
		poseEstimators[i]->initializeClusterExtractor(200,2500,0.01);
	}

	perceptionPaused=false;
	ROS_WARN("Resuming Perception Engine");


}


int main(int argc, char* argv[]){

	perceptionPaused = false;

	ros::init(argc, argv, "PoseEstimation6D");
	ros::NodeHandle nh;
	/** configuration */
	int minClusterSize, maxClusterSize;
	double clusterTolerance;
	bool publishApproximatePoses;
	int maxIterations;
	float maxCorrespondenceThreshold;
	float reliableScoreThreshold;
	ros::param::param<int>("/poseEstimator6D/minClusterSize", minClusterSize, 100);
	ros::param::param<int>("/poseEstimator6D/maxClusterSize", maxClusterSize, 2000);
	ros::param::param<double>("/poseEstimator6D/clusterTolerance", clusterTolerance, 0.01);	//1 cm by default
	ros::param::param<bool>("/poseEstimator6D/publishApproximatePoses", publishApproximatePoses, true);	//1 cm by default

	noOfRegions = 0;
	maxNoOfObjects = 0;
//	if (nh.hasParam("minClusterSize")){
//		nh.getParam("minClusterSize", minClusterSize);
//	}

	if(argc < 4){


		ROS_ERROR("Not enough arguments:\n Usage:\t "
				"rosrun poseEstimator6D <no_of_regions> <maxNoOfObjects> <region_1_config_file> "
				"<region_2_config_file>.... <region_label_1> <region_label_2>\n"
				"\nExample: rosrun poseEstimator6D 2 3 redRoiConfig.cfg greenRoiConfig.cfg red green");
		exit(0);
	} else if(argc != (3+ 2*atoi(argv[1]))){
		ROS_ERROR("Not enough arguments:\n Usage:\t "
				"rosrun colorBasedRoiExtractor <no_of_regions> <maxNoOfObjects> <region_1_config_file> "
				"<region_2_config_file>.... <region_label_1> <region_label_2>");
		exit(0);
	}
	noOfRegions = atoi(argv[1]);
	maxNoOfObjects = atoi(argv[2]);

	ROS_INFO("Finding pose for at most [%d] object(s) in [%d] color based extracted regions",
			maxNoOfObjects, noOfRegions);

	BRICS_3D::PointCloud3D* tmpModel = new BRICS_3D::PointCloud3D;
	tmpModel->readFromTxtFile("test_model_green_1.txt");
	modelDataBase.push_back(tmpModel);
	modelNames.push_back("test_model_green_1");

	//define the HSV limit variables;
	float minLimitH[noOfRegions], minLimitS[noOfRegions],
	maxLimitH[noOfRegions], maxLimitS[noOfRegions];

	//parse the configuration files and set up the HSV limits and set up the pose estimators
	std::ifstream configFileStream;
	for (unsigned int i = 0; i< noOfRegions; i++){

		poseEstimators.push_back(new BRICS_3D::PoseEstimation6D());

		configFileStream.open(argv[i+3]);
		if ( configFileStream.is_open() ) {     //if file exists
			std::string s;
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
			ROS_ERROR("Configuration file: %s not found!!", argv[i+3]);
			exit(0);
		}

		//Setting the max-no of objects to be searched for
		poseEstimators[i]->setMaxNoOfObjects(maxNoOfObjects);
		//Setting the label which will be used to publish the transforms
		poseEstimators[i]->setRegionLabel(argv[3+noOfRegions+i]);
		//Initializing the limits in HSV space to extract the ROI
		poseEstimators[i]->initializeLimits(minLimitH[i], maxLimitH[i], minLimitS[i], maxLimitS[i]);
		//Initializing the cluster extractor limits
		ROS_INFO("Object Clustering Parametrs: [%d] [%d] [%f]", minClusterSize, maxClusterSize, clusterTolerance);
		poseEstimators[i]->initializeClusterExtractor(minClusterSize,maxClusterSize,clusterTolerance);
		poseEstimators[i]->setPublishingStatus(publishApproximatePoses);

		//Add models to model database
		assert(modelDataBase.size() == modelNames.size());
		for (unsigned int j = 0; j < modelDataBase.size(); ++j) {
			poseEstimators[i]->addModelToDataBase(modelDataBase[j], modelNames[j]);
		}
	}


	//subscribe to perception engine control messsage
	ros::Subscriber  perceptionControlSubscriber;
	perceptionControlSubscriber= nh.subscribe("/perceptionControl", 1,&perceptionControlCallback);

	//subscribe to perception engine configuration messsage
	ros::Subscriber  perceptionConfigurationSubscriber;
	perceptionConfigurationSubscriber= nh.subscribe("/perceptionConfiguration", 1,&perceptionConfigurationCallback);


	//subscribe to kinect point cloud messages
	ros::Subscriber  kinectCloudSubscriber[noOfRegions];
	for (unsigned int i = 0; i < noOfRegions ; i++){
		kinectCloudSubscriber[i]= nh.subscribe("/camera/rgb/points", 1,&kinectCloudCallback);
		poseEstimators[i]->setMaxNoOfObjects(maxNoOfObjects);
	}

	ros::spin();

	return 0;
}
