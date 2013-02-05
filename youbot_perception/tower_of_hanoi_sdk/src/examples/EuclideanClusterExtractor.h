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

#ifndef EUCLIDEANCLUSTEREXTRACTION_H_
#define EUCLIDEANCLUSTEREXTRACTION_H_

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <stdio.h>
#include <vector>

//#include "algorithm/segmentation/EuclideanClustering.h"
#include "EuclideanClustering.h"
#include "brics_3d/algorithm/featureExtraction/Centroid3D.h"
#include "brics_3d/util/PCLTypecaster.h"

//Todo add the comments
namespace brics_3d {

class EuclideanClusterExtractor {
private:

	/**
	 * Publisher to output the extracted clusters
	 */
	ros::Publisher *extractedClusterPublisher;

	/**
	 * Utility  object to type-cast data-types between brics_3d and PCL
	 */
	brics_3d::PCLTypecaster pclTypecaster;

	/**
	 * Object for extracting euclidean clusters
	 */
	brics_3d::SDK::EuclideanClustering euclideanClusterExtractor;

	/**
	 * Object for estimating 3D centroids of the estimated object clusters
	 */
	brics_3d::Centroid3D* centroid3DEstimator;

	/**
	 * Maximum number of object clusters which will be published.
	 */
	int maxNoOfObjects;

public:
	EuclideanClusterExtractor();
	virtual ~EuclideanClusterExtractor();

	/**
	 * Callback function for Kinect data. Finds the ROI from the input data
	 * @param cloud input cloud from Kinect
	 */
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);


	/**
	 * Set the pointer to an array of publisher to be used to output the estimated object clusters
	 * @param extractedClusterPublisher Publisher to be used to output the estimated object clusters
	 */
	void setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher);


	/**
	 *
	 * @return an array of publisher to be used to output the estimated object clusters
	 */
	ros::Publisher *getExtractedClusterPublisher() const;


	/**
	 *
	 * @return the maximum number of objects to be published
	 */
	int getMaxNoOfObjects() const;


	/**
	 * Sets the maximum number of objects to be published
	 * @param maxNoOfObjects maximum number of objects to be published
	 */
	void setMaxNoOfObjects(int maxNoOfObjects);


	/**
	 * Set the minimum cluster size to be used for estimating the clusters
	 * @param size minimum cluster size to be used for estimating the clusters
	 */
	inline void setMinClusterSize(int size){
		this->euclideanClusterExtractor.setMinClusterSize(size);
	}

	/**
	 *
	 * @return minimum cluster size to be used for estimating the clusters
	 */
	inline int getMinCLusterSize(){
		return this->euclideanClusterExtractor.getMinClusterSize();
	}


	/**
	 * Set the maximum cluster size to be used for estimating the clusters
	 * @param size maximum cluster size to be used for estimating the clusters
	 */
	inline void setMaxClusterSize(int size){
		this->euclideanClusterExtractor.setMaxClusterSize(size);
	}


	/**
	 * get the maximum cluster size currently used by the cluster estimator
	 * @return maximum cluster size to be used for estimating the clusters
	 */
	inline int getMaxClusterSize(){
		return this->euclideanClusterExtractor.getMaxClusterSize();
	}


	/**
	 * get the minimum cluster size currently used by the cluster estimator
	 * @return minimum cluster size to be used for estimating the clusters
	 */

	inline void setClusterTolerance(float tolerance){
		this->euclideanClusterExtractor.setClusterTolerance(tolerance);
	}


	/**
	 * get the current cluster tolerance limit
	 * @return the current cluster tolerance limit
	 */
	inline float getClusterTolerance(){
		return	this->euclideanClusterExtractor.getClusterTolerance();
	}


	/**
	 * Initialize the cluster extractor and set up the cluster-publishers
	 * @param maxNoObjects max number valid objects possible to be found in the input cloud
	 * @param extractedClusterPublisher ros publisher to be used to publish the found clusters
	 * @param minClusterSize minimum number of points required to consider a cluster to be valid
	 * @param maxClusterSize maximum number of points in a cluster allowed
	 * @param clusterTolerance the euclidean distance  based cluster tolerance limit
	 */
	void initializeExtractor(int maxNoObjects, ros::Publisher *extractedClusterPublisher,
			int minClusterSize, int maxClusterSize, float clusterTolerance);

};

}

#endif /* EUCLIDEANCLUSTEREXTRACTION_H_ */
