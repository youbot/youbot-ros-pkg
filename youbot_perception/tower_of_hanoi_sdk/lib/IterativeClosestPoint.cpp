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

#include "IterativeClosestPoint.h"

namespace BRICS_3D {

namespace SDK {

IterativeClosestPoint::IterativeClosestPoint() {
	distance = 0.1;
	transformationEpsilon = 1e-6;
	maxIterations = 1000;
}


IterativeClosestPoint::~IterativeClosestPoint() {}


void IterativeClosestPoint::estimateBestFit(BRICS_3D::PointCloud3D *inCloud, BRICS_3D::PointCloud3D *outCloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	BRICS_3D::PCLTypecaster pclTypecaster;

	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr objectModelPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> Final;

	pclTypecaster.convertToPCLDataType(targetCloudPtr, inCloud);
	//    std::cout << "object model cloud size: " << objectModel->getSize()<< std::endl;
	pclTypecaster.convertToPCLDataType(objectModelPtr, objectModel);

	icp.setInputCloud(objectModelPtr);
	icp.setInputTarget(targetCloudPtr);
	icp.setMaxCorrespondenceDistance(distance);
	icp.setTransformationEpsilon (transformationEpsilon);
	icp.setMaximumIterations (maxIterations);



	int count=0;
	float scoreEpsilon = 0.000008;
	while( (count<10) & (icp.getFitnessScore() > scoreEpsilon) ) {
		icp.align(Final);
		count++;
	}
	this->fitnessScore=icp.getFitnessScore();
	this->finalTransformation = icp.getFinalTransformation();


	pcl::PointCloud<pcl::PointXYZ> ::Ptr FinalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	FinalCloudPtr->width = Final.width;
	FinalCloudPtr->height = Final.height;
	FinalCloudPtr->points.resize(Final.height * Final.width);
	for (size_t i = 0; i<FinalCloudPtr->size(); i++){
		FinalCloudPtr->points[i].x = Final.points[i].x;
		FinalCloudPtr->points[i].y = Final.points[i].y;
		FinalCloudPtr->points[i].z = Final.points[i].z;
	}
	pclTypecaster.convertToBRICS3DDataType(FinalCloudPtr, outCloud);


}

}

}
