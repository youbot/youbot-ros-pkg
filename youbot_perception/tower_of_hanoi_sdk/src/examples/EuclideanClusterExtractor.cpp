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
#include "EuclideanClusterExtractor.h"

namespace brics_3d{

EuclideanClusterExtractor::EuclideanClusterExtractor() {
	centroid3DEstimator = new brics_3d::Centroid3D();
	maxNoOfObjects = 3;
}

ros::Publisher *EuclideanClusterExtractor::getExtractedClusterPublisher() const
{
	return extractedClusterPublisher;
}

int EuclideanClusterExtractor::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

void EuclideanClusterExtractor::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;
}

void EuclideanClusterExtractor::setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher){
	this->extractedClusterPublisher = extractedClusterPublisher;
}

EuclideanClusterExtractor::~EuclideanClusterExtractor() {
	delete centroid3DEstimator;
}


void EuclideanClusterExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	//ToDo assert if every thing is initialized or not

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());





    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
    pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

    if(cloud_xyz_rgb_ptr->size() > this->euclideanClusterExtractor.getMinClusterSize()){
        brics_3d::PointCloud3D *in_cloud = new brics_3d::PointCloud3D();
        std::vector<brics_3d::PointCloud3D*> extracted_clusters;
    // cast PCL to brics_3d type
    pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);

    //extract the clusters
    euclideanClusterExtractor.extractClusters(in_cloud, &extracted_clusters);
    ROS_INFO("No of clusters found: %d", extracted_clusters.size());
    //Publish the extracted clusters
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
	int regions;
	if(extracted_clusters.size() < abs(maxNoOfObjects)) {
		regions = extracted_clusters.size();
	} else {
		regions = maxNoOfObjects;
	}
    for (int i = 0; i < regions; i++){

    	if(extracted_clusters[i]->getSize() > 0){
    	Eigen::Vector3d centroid3d = centroid3DEstimator->computeCentroid(extracted_clusters[i]);

    	pclTypecaster.convertToPCLDataType(tempCloud, extracted_clusters[i]);
    	tempCloud->header.frame_id = "/openni_rgb_optical_frame";
    	extractedClusterPublisher[i].publish(*tempCloud);

    	ROS_INFO("Cluster Position 3D: [%f, %f, %f]", centroid3d[0], centroid3d[1], centroid3d[2]);

         static tf::TransformBroadcaster br;
         tf::Transform transform;
         transform.setOrigin( tf::Vector3(centroid3d[0], centroid3d[1], centroid3d[2]) );
         //Todo stop using Quaternion
         transform.setRotation( tf::Quaternion(0, 0, 0) );
         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
        		 extractedClusterPublisher[i].getTopic()));
    	}
    }

    tempCloud.reset();
    delete(in_cloud);
    extracted_clusters.clear();
    }

}

void EuclideanClusterExtractor::initializeExtractor(int maxNoObjects,
		ros::Publisher *extractedClusterPublisher, int minClusterSize, int maxClusterSize,
																		float clusterTolerance){

	setMinClusterSize(minClusterSize);
	setMaxClusterSize(maxClusterSize);
	setClusterTolerance(clusterTolerance);
	setMaxNoOfObjects(maxNoObjects);
	setExtractedClusterPublisher(extractedClusterPublisher);

}

}
