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

#ifndef POSEESTIMATION6DEXAMPLE_H_
#define POSEESTIMATION6DEXAMPLE_H_

//ros-headers
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <tf/transform_broadcaster.h>

//BRICS_3D headers
#include "IterativeClosestPoint.h"
#include "util/SimplePointCloudGeneratorCube.h"
#include "util/PCLTypecaster.h"
#include "core/PointCloud3D.h"
#include "algorithm/featureExtraction/Centroid3D.h"
#include "core/HomogeneousMatrix44.h"

//PCL Headers
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/common/transform.h>


#include <Eigen/Geometry>


namespace BRICS_3D {

class ModelFitting {

	/**
	 * Object for performing ICP
	 */
	BRICS_3D::SDK::IterativeClosestPoint* poseEstimatorICP;

	/**
	 * two-sided cube model
	 */
	BRICS_3D::PointCloud3D *cube2D;

	/**
	 * Three sided cube model
	 */
	BRICS_3D::PointCloud3D *cube3D;

	/**
	 * Object to create the cube models
	 */
	BRICS_3D::SimplePointCloudGeneratorCube cubeModelGenerator;

	/**
	 * Object to typecast data-types between PCL and BRICS_3D
	 */
	BRICS_3D::PCLTypecaster pclTypecaster;

	/**
	 * Fitting score threshold indicating a good match
	 */
	float reliableScoreThreshold;

	/**
	 * Publisher to output the fitted models
	 */
	ros::Publisher *modelPublisher;

	/**
	 * Best score found till now
	 */
	float bestScore;

	/**
	 * Best translation found   by the model fitting process
	 */
	float xtranslation;
	float ytranslation;
	float ztranslation;

	Eigen::Matrix4f* bestTransformation;

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

	ModelFitting();
	virtual ~ModelFitting();

	/**
	 * Callback module to accept the input cloud
	 * @param cloud	input cloud
	 */
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);


	/**
	 *	Set the publishers
	 * @param pub	Array of ROS::Publishers to which the fitted models will be published
	 */
	void setModelPublisher(ros::Publisher *pub){
		this->modelPublisher = pub;
	}

};

}

#endif /* POSEESTIMATION6DEXAMPLE_H_ */
