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

#include "PoseEstimation6D.h"
#include <limits>

namespace BRICS_3D {

PoseEstimation6D::PoseEstimation6D() {
	//default initialization of roi extractor
	this->hsvBasedRoiExtractor.setMinH(0);
	this->hsvBasedRoiExtractor.setMaxH(0);
	this->hsvBasedRoiExtractor.setMinS(0);
	this->hsvBasedRoiExtractor.setMaxS(0);

	//default initialization of cluster extractor
	this->euclideanClusterExtractor.setMinClusterSize(0);
	this->euclideanClusterExtractor.setMaxClusterSize(0);
	this->euclideanClusterExtractor.setClusterTolerance(0);

	poseEstimatorICP = new BRICS_3D::SDK::IterativeClosestPoint();

	centroid3DEstimator =  new BRICS_3D::Centroid3D();

	//default initialization of cube models
	cube2D = new BRICS_3D::PointCloud3D();
	cube3D = new BRICS_3D::PointCloud3D();

	cubeModelGenerator.setPointsOnEachSide(5);
	cubeModelGenerator.setCubeSideLength(0.06);

	cubeModelGenerator.setNumOfFaces(2);
	cubeModelGenerator.generatePointCloud(cube2D);

	cubeModelGenerator.setNumOfFaces(3);
	cubeModelGenerator.generatePointCloud(cube3D);

	modelDatabase.clear();
	modelNames.clear();
	addModelToDataBase(cube2D, "cube_with_two_faces");
	addModelToDataBase(cube3D, "cube_with_three_faces");

	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(90,0,0,0,0,0,tempHomogenousMatrix,true);
	BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(
			tempHomogenousMatrix(0), tempHomogenousMatrix(4), tempHomogenousMatrix(8),
			tempHomogenousMatrix(1), tempHomogenousMatrix(5), tempHomogenousMatrix(9),
			tempHomogenousMatrix(2), tempHomogenousMatrix(6), tempHomogenousMatrix(10),
			0,0,0);

	cube2D->homogeneousTransformation(homogeneousTrans);
	cube3D->homogeneousTransformation(homogeneousTrans);

	reliableScoreThreshold = 0.00008;
	maxIterations = 1000;
	maxCorrespondenceThreshold = 0.1;

	maxNoOfObjects = 0;
	this->publishApproximatePoses = true;

	ROS_INFO("Initialization done.");
}

int PoseEstimation6D::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

std::string PoseEstimation6D::getRegionLabel() const
{
	return regionLabel;
}

void PoseEstimation6D::setRegionLabel(std::string regionLabel)
{
	this->regionLabel = regionLabel;
}

void PoseEstimation6D::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;

	int i;
	for (i = 0; i < maxNoOfObjects; ++i) {
		bestScore.push_back(1000);
		xtranslation.push_back(0);
		ytranslation.push_back(0);
		ztranslation.push_back(0);
		bestTransformation.push_back(new Eigen::Matrix4f());
	}

}

PoseEstimation6D::~PoseEstimation6D() {
	delete cube2D;
	delete cube3D;
	delete poseEstimatorICP;
	delete centroid3DEstimator;
}

void PoseEstimation6D::initializeLimits(float minLimitH, float maxLimitH, float minLimitS,
		float maxLimitS){
	this->hsvBasedRoiExtractor.setMinH(minLimitH);
	this->hsvBasedRoiExtractor.setMaxH(maxLimitH);
	this->hsvBasedRoiExtractor.setMinS(minLimitS);
	this->hsvBasedRoiExtractor.setMaxS(maxLimitS);
}

void PoseEstimation6D::initializeClusterExtractor(int minClusterSize, int maxClusterSize,
		float clusterTolerance){

	this->euclideanClusterExtractor.setMinClusterSize(minClusterSize);
	this->euclideanClusterExtractor.setMaxClusterSize(maxClusterSize);
	this->euclideanClusterExtractor.setClusterTolerance(clusterTolerance);
}

void PoseEstimation6D::initializeModelFitting(int maxIterations, float maxCorrespondenceThreshold, float reliableScoreThreshold) {
	this->maxIterations = maxIterations;
	this->maxCorrespondenceThreshold = maxCorrespondenceThreshold;
	this->reliableScoreThreshold = reliableScoreThreshold;
}

void PoseEstimation6D::estimatePose(BRICS_3D::PointCloud3D *in_cloud, int objCount){

	Eigen::Vector3d centroid3d = centroid3DEstimator->computeCentroid(in_cloud);
	bool reliableModelFound=true;
	float xTrans = centroid3d[0];
	float yTrans = centroid3d[1];
	float zTrans = centroid3d[2];



	//	ROS_INFO("Initial estimate \n\tTranslation-[x,y,z,]=[%f,%f,%f]",xTrans, yTrans, zTrans);

	//Translate the cube models which will be our initial estimate for ICP
	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(0,0,0,xTrans,yTrans,zTrans,tempHomogenousMatrix,true);
	BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			xTrans,yTrans,zTrans);

	std::vector<BRICS_3D::PointCloud3D*> transformedModelDatabase;

	for (unsigned int index = 0; index < modelDatabase.size(); ++index) {
		transformedModelDatabase.push_back(new BRICS_3D::PointCloud3D());
		for(unsigned int i=0; i< (*modelDatabase[index]).getSize(); i++){
			BRICS_3D::Point3D *tempPoint = new BRICS_3D::Point3D((*modelDatabase[index]).getPointCloud()->data()[i].getX(),
					(*modelDatabase[index]).getPointCloud()->data()[i].getY(),
					(*modelDatabase[index]).getPointCloud()->data()[i].getZ());
			(*transformedModelDatabase[index]).addPoint(tempPoint);
			delete tempPoint;
		}
		(*transformedModelDatabase[index]).homogeneousTransformation(homogeneousTrans);
	}
	assert(modelDatabase.size() == transformedModelDatabase.size());
	std::cout << "Model database size = " << modelDatabase.size() << std::endl;

	std::vector<BRICS_3D::PointCloud3D*> finalModels;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Vector4f> > finalTransformations;
	std::map<float, int> scoreToIndexMapping;
	std::map<float, int>::const_iterator scoreToIndexMappingIterator;

	poseEstimatorICP->setDistance(maxCorrespondenceThreshold);
	poseEstimatorICP->setMaxIterations(maxIterations);
	ROS_INFO("[%s_%d] Results for fitted models: index | score    | scoreThresh | name", regionLabel.c_str(), objCount);
	for (unsigned int index = 0; index < modelDatabase.size(); ++index) {
		finalModels.push_back(new BRICS_3D::PointCloud3D());

		//Performing model alignment
		poseEstimatorICP->setObjectModel(transformedModelDatabase[index]);
		poseEstimatorICP->estimateBestFit(in_cloud, finalModels[index]);
		float score = poseEstimatorICP->getFitnessScore();
		scoreToIndexMapping.insert(std::make_pair(score, index));
		finalTransformations.push_back(poseEstimatorICP->getFinalTransformation());
		ROS_INFO("[%s_%d]                                %i | %f | %f    | %s ", regionLabel.c_str(), objCount, index, score, reliableScoreThreshold, modelNames[index].c_str());
	}


	// real sorting might be better
	double smallestError = std::numeric_limits<double>::max();
	int smallestErrorIndex = 0;
	for (scoreToIndexMappingIterator = scoreToIndexMapping.begin(); scoreToIndexMappingIterator != scoreToIndexMapping.end(); ++scoreToIndexMappingIterator) {
		if (scoreToIndexMappingIterator->first < smallestError) {
			smallestError = scoreToIndexMappingIterator->first;
			smallestErrorIndex = scoreToIndexMappingIterator->second;
		}
	}

	scoreToIndexMappingIterator = scoreToIndexMapping.find(smallestError);
	assert(scoreToIndexMappingIterator != scoreToIndexMapping.end());
	if(scoreToIndexMappingIterator->first < bestScore[objCount]){
		if(scoreToIndexMappingIterator->first > reliableScoreThreshold){
			ROS_INFO("[%s_%d] Approximate Model Found. Object May Not be visible enough...", regionLabel.c_str(), objCount);
			reliableModelFound=false;
		} else {
			ROS_INFO("[%s_%d] Reliable Model Found", regionLabel.c_str(), objCount);
//			std::stringstream fileName;
//			fileName.str("");
//			fileName << "test_model_" << regionLabel.c_str() << "_" << objCount << ".txt";
//			in_cloud->storeToTxtFile(fileName.str());
		}
		*(bestTransformation[objCount]) = finalTransformations[scoreToIndexMappingIterator->second];
		centroid3d = centroid3DEstimator->computeCentroid(finalModels[scoreToIndexMappingIterator->second]);
		xtranslation[objCount]=centroid3d[0];
		ytranslation[objCount]=centroid3d[1];
		ztranslation[objCount]=centroid3d[2];
	}
	ROS_INFO("[%s_%d] Best score found for model %i : %f", regionLabel.c_str(), objCount, scoreToIndexMappingIterator->second, scoreToIndexMappingIterator->first);
	bestScore[objCount] = scoreToIndexMappingIterator->first;

    double yRot = asin (-(*(bestTransformation[objCount]))(2));
    double xRot = asin ((*(bestTransformation[objCount]))(6)/cos(yRot));
    double zRot = asin ((*(bestTransformation[objCount]))(1)/cos(yRot));

//	Eigen::Matrix4f  tempHomogenousMatrix;
//  calculateHomogeneousMatrix(xRot, yRot, zRot, translation[0], translation[1], translation[2],tempHomogenousMatrix,0);

    static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(xtranslation[objCount], ytranslation[objCount], ztranslation[objCount]) );
     //Todo stop using Quaternion
     transform.setRotation( tf::Quaternion(xRot, yRot, zRot) );
     std::stringstream ss;
     ss << regionLabel << "_object_" << objCount;
     if(publishApproximatePoses){
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
    		 ss.str()));
     } else {
    	 if(reliableModelFound){
    		 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
    		     		 ss.str()));
    	 }
     }


	for (unsigned int index = 0; index < finalModels.size(); ++index) {
		delete finalModels[index];
	}
	for (unsigned int index = 0; index < transformedModelDatabase.size(); ++index) {
		delete transformedModelDatabase[index];
	}

}

void PoseEstimation6D::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	std::cout << "Estimating Pose 6D :) :)"<< std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	BRICS_3D::ColoredPointCloud3D *in_cloud = new BRICS_3D::ColoredPointCloud3D();
	BRICS_3D::PointCloud3D *color_based_roi = new BRICS_3D::ColoredPointCloud3D();
	std::vector<BRICS_3D::PointCloud3D*> extracted_clusters;

	//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

	// cast PCL to BRICS_3D type
	pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
	ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());

	if(in_cloud->getSize() < euclideanClusterExtractor.getMinClusterSize()) return;
	/**==================================================================================================== [color based roi extraction]**/
	//perform HSV color based extraction
	hsvBasedRoiExtractor.extractColorBasedROI(in_cloud, color_based_roi);
	ROS_INFO("[%s] Size of extracted cloud : %d ", regionLabel.c_str(),color_based_roi->getSize());
	if(color_based_roi->getSize() < euclideanClusterExtractor.getMinClusterSize()) return;
	/**==================================================================================================== [cluster extraction]**/
	//extract the clusters
	euclideanClusterExtractor.extractClusters(color_based_roi, &extracted_clusters);
	ROS_INFO("[%s] No of clusters found: %d", regionLabel.c_str(), extracted_clusters.size());


	/**==================================================================================================== [6D pose estimation]**/
	//Estimate 6D Pose
	int regions;
	if(extracted_clusters.size() < abs(maxNoOfObjects)) {
		regions = extracted_clusters.size();
	} else {
		regions = maxNoOfObjects;
	}

	for (int i = 0; i < regions; i++){
		if(extracted_clusters[i]->getSize() > 0){
			//Estimate Pose
			estimatePose(extracted_clusters[i],i+1);
		}
	}


	delete in_cloud;
	delete color_based_roi;
	extracted_clusters.clear();

}

void PoseEstimation6D::addModelToDataBase(BRICS_3D::PointCloud3D* model, std::string name) {
	if (model->getSize() == 0) {
		ROS_WARN("Model point cloud to be added to database is empty. Possibly the model was not loaded correctly.");
		return;
	}

	/* Demean the model point cloud (should be already provided/assumed?) */
	Eigen::Vector3d centroid3d = centroid3DEstimator->computeCentroid(model);
	float xTrans = centroid3d[0];
	float yTrans = centroid3d[1];
	float zTrans = centroid3d[2];

	ROS_DEBUG("Model centroid is transalated to origin by (%f, %f, %f)", -xTrans, -yTrans, -zTrans);
	BRICS_3D::HomogeneousMatrix44* demeanTranslation = new HomogeneousMatrix44(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			-xTrans,-yTrans,-zTrans);
	model->homogeneousTransformation(demeanTranslation);

	modelDatabase.push_back(model);
	modelNames.push_back(name);
	ROS_INFO("Added a new model %s with %i points to database. Database has now %i models.", name.c_str(), model->getSize(), modelDatabase.size());
	assert(modelDatabase.size() == modelNames.size());
}

}
