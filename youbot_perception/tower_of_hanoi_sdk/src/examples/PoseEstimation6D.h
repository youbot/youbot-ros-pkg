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

#ifndef POSEESTIMATION6D_H_
#define POSEESTIMATION6D_H_

//ros headers
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <tf/transform_broadcaster.h>

//BRICS_3D headers
#include "algorithm/filtering/ColorBasedROIExtractorHSV.h"
#include "util/PCLTypecaster.h"
#include "core/ColoredPointCloud3D.h"
#include "EuclideanClustering.h"
#include "algorithm/featureExtraction/Centroid3D.h"
#include "core/HomogeneousMatrix44.h"
#include "util/SimplePointCloudGeneratorCube.h"
#include "IterativeClosestPoint.h"


//standard headers
#include <iostream>
#include <vector>
#include <Eigen/StdVector>
namespace BRICS_3D {

class PoseEstimation6D {

	/**
	 * Maximum number of objects to be searched for.
	 */
	int maxNoOfObjects;

	/**
	 * The label that will be used to publish the transforms
	 */
	std::string regionLabel;

	/**
	 * object for extracting ROIs based on HSV-color space limits
	 */
	BRICS_3D::ColorBasedROIExtractorHSV hsvBasedRoiExtractor;

	/**
	 * Utility object for type-casting data between BRICS_3D and PCL
	 */
	BRICS_3D::PCLTypecaster pclTypecaster;

	/**
	 * Indicates if the color based ROI extractor is intialized with proper limits
	 */
	bool initializedRoiExtractor;

	/**
	 * Object for extracting euclidean clusters
	 */
	BRICS_3D::SDK::EuclideanClustering euclideanClusterExtractor;

	/**
	 * Object for estimating 3D centroids of the estimated object clusters
	 */
	BRICS_3D::Centroid3D *centroid3DEstimator;

	/**
	 * two-sided cube model
	 */
	BRICS_3D::PointCloud3D *cube2D;

	/**
	 * Three sided cube model
	 */
	BRICS_3D::PointCloud3D *cube3D;

	/**
	 * data base that holds all pont clouds representing the models
	 */
	std::vector<BRICS_3D::PointCloud3D*> modelDatabase;

	/**
	 * Vector that hold the associated names to the models.
	 */
	std::vector<std::string> modelNames;

	/**
	 * Object to create the cube models
	 */
	BRICS_3D::SimplePointCloudGeneratorCube cubeModelGenerator;


	/**
	 * Fitting score threshold indicating a good match
	 */
	float reliableScoreThreshold;

	/**
	 * Max interations for ICP
	 */
	int maxIterations;

	/**
	 * Maximum distonce correspondence threshold for ICP
	 */
	float maxCorrespondenceThreshold;

	/**
	 * Best score found till now
	 */
	std::vector<float> bestScore;

	/**
	 * Object for performing ICP
	 */
	BRICS_3D::SDK::IterativeClosestPoint *poseEstimatorICP;


	/**
	 * Best translation found   by the model fitting process
	 */
	std::vector<float> xtranslation;
	std::vector<float> ytranslation;
	std::vector<float> ztranslation;

	/**
	 * Indicates if all the pose estimates should be published or only the reliable estimates
	 */
	bool publishApproximatePoses;

	/**
	 * Best transformation found by the model fitting process
	 */
	std::vector<Eigen::Matrix4f*> bestTransformation;

	/**
	 * Helper function to calculate a homogeneous matrix for affine-transformation
	 * @param xRot		Rotation along x-axis
	 * @param yRot		Rotation along y-axis
	 * @param zRot		Rotation along z-axis
	 * @param xtrans	Translation along x-axis
	 * @param ytrans	Translation along y-axis
	 * @param ztrans	Translation along z-axis
	 * @param homogeneousMatrix	Output matrix
	 * @param inDegrees	Indicates whether the rotation angles are in degree(1) or radian(0)
	 */
	void calculateHomogeneousMatrix(float xRot,float yRot,
			float zRot, float xtrans, float ytrans, float ztrans, Eigen::Matrix4f  &homogeneousMatrix, bool inDegrees){
		if(inDegrees){
			float PI = 3.14159265;
			xRot = xRot*(PI/180);
			yRot *= yRot*(PI/180);
			zRot *= zRot*(PI/180);
		}
		/*
		 * layout:
		 * 0 4 8  12
		 * 1 5 9  13
		 * 2 6 10 14
		 * 3 7 11 15
		 */


		homogeneousMatrix(3)=0;
		homogeneousMatrix(7)=0;
		homogeneousMatrix(11)=0;
		homogeneousMatrix(15)=1;

	    //translation
	    homogeneousMatrix(12)=xtrans;
		homogeneousMatrix(13)=ytrans;
		homogeneousMatrix(14)=ztrans;

		//rotation
		homogeneousMatrix(0) = cos(yRot)*cos(zRot);
		homogeneousMatrix(1) = cos(yRot)*sin(zRot);
		homogeneousMatrix(2) = -sin(yRot);

		homogeneousMatrix(4) = -cos(xRot)*sin(zRot) + sin(xRot)*sin(yRot)*cos(zRot);
		homogeneousMatrix(5) = cos(xRot)*cos(zRot) + sin(xRot)*sin(yRot)*sin(zRot);
		homogeneousMatrix(6) = sin(xRot)*cos(yRot);

		homogeneousMatrix(8) = sin(xRot)*sin(zRot) + cos(xRot)*sin(yRot)*cos(zRot);
		homogeneousMatrix(9) = -sin(xRot)*cos(zRot) + cos(xRot)*sin(yRot)*sin(zRot);
		homogeneousMatrix(10)= cos(xRot)*cos(yRot);


	}

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PoseEstimation6D();
	virtual ~PoseEstimation6D();
    int getMaxNoOfObjects() const;
    void setMaxNoOfObjects(int maxNoOfObjects);

	/**
	 * Callback function for Kinect data.
	 * @param cloud input cloud from Kinect
	 */
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);


	/**
	 * Initializes the HSV color-space based ROI extractor with Hue ans Saturation limits
	 * @param minLimitH	minimum Hue value
	 * @param maxLimitH maximum Hue value
	 * @param minLimitS minimum saturation value
	 * @param maxLimitS maximum saturation value
	 *
	 */
	void initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS);

	void initializeClusterExtractor(int minClusterSize, int maxClusterSize, float clusterTolerance);

	void initializeModelFitting(int maxIterations, float maxCorrespondenceThreshold, float reliableScoreThreshold);

	void estimatePose(BRICS_3D::PointCloud3D *in_cloud, int objCount);

	std::string getRegionLabel() const;
    void setRegionLabel(std::string regionLabel);
    void setPublishingStatus(bool publishApproximatePoses){
    	this->publishApproximatePoses=publishApproximatePoses;
    }

    /**
     * Add a new model - represented as point cloud - to the database.
     * All models will in the database will be fit into a region of intrest and the one with a
     * @param model The new model the will be added to the database
     * @param name A name that helps to identify that model
     */
    void addModelToDataBase(BRICS_3D::PointCloud3D* model, std::string name);
};

}

#endif /* POSEESTIMATION6D_H_ */
