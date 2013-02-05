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

#include "EuclideanClustering.h"

#include "brics_3d/util/PCLTypecaster.h"
#include "brics_3d/core/ColorSpaceConvertor.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/point_types.h"
#include "pcl/features/normal_3d.h"

namespace brics_3d {
namespace SDK{
EuclideanClustering::EuclideanClustering() {
	// TODO Auto-generated constructor stub

	this-> minClusterSize =0;
	this->maxClusterSize =0;
	this->clusterTolerance =0;
}

EuclideanClustering::~EuclideanClustering() {
	// TODO Auto-generated destructor stub
}

void EuclideanClustering::extractClusters(brics_3d::PointCloud3D *inCloud,
		std::vector<brics_3d::PointCloud3D*> *extractedClusters){

//	printf("[EuclideanClusterExtraction.cpp][checkpoint] size of input cloud is %d\n", inCloud->getSize());

	brics_3d::PCLTypecaster pclTypecaster;

	//converting input cloud into PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPclPtr(new pcl::PointCloud<pcl::PointXYZ> ());
	pclTypecaster.convertToPCLDataType(inCloudPclPtr, inCloud);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (inCloudPclPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtractor;
	euclideanClusterExtractor.setClusterTolerance (this->clusterTolerance); // 2cm
	euclideanClusterExtractor.setMinClusterSize (this->minClusterSize);
	euclideanClusterExtractor.setMaxClusterSize (this->maxClusterSize);
	euclideanClusterExtractor.setSearchMethod (tree);
	euclideanClusterExtractor.setInputCloud(inCloudPclPtr);
	euclideanClusterExtractor.extract (cluster_indices);

//	printf("Parameters used or extraction:\n \t Min Cluster Size = %d\n"
//		"\tMax Cluster Size=%d\n"
//		"\tCluster Tolerance=%f\n "
//		"\tNumber of objects found: %d\n", this->minClusterSize,
//			this->maxClusterSize, this->clusterTolerance, cluster_indices.size());

//	//Building up the pointclouds for corresponding clusters
	int index =0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

		extractedClusters->push_back(new brics_3d::PointCloud3D());

		extractedClusters->data()[index]->getPointCloud()->clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			extractedClusters->data()[index]->addPoint(new Point3D(	inCloudPclPtr->points[*pit].x,
													inCloudPclPtr->points[*pit].y,
													inCloudPclPtr->points[*pit].z) );
		}
	//	printf("[EuclideanClusterExtraction.cpp][checkpoint] size of cluster is %d\n", extractedClusters->data()[index]->getSize());
		index++;
	}
}


//void EuclideanClustering::extractClusters(brics_3d::ColoredPointCloud3D *inCloud,
//		std::vector<brics_3d::ColoredPointCloud3D*> *extractedClusters){
//
//	brics_3d::PCLTypecaster pclTypecaster;
//	brics_3d::ColorSpaceConvertor colorSpaceConvertor;
//	//converting input cloud into PCL format
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPclPtr(new pcl::PointCloud<pcl::PointXYZRGB> ());
//	pclTypecaster.convertToPCLDataType(inCloudPclPtr, inCloud);
//
//	// Creating the KdTree object for the search method of the extraction
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//	tree->setInputCloud (inCloudPclPtr);
//	std::vector<pcl::PointIndices> cluster_indices;
//	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclideanClusterExtractor;
//	euclideanClusterExtractor.setClusterTolerance (this->clusterTolerance); // 2cm
//	euclideanClusterExtractor.setMinClusterSize (this->minClusterSize);
//	euclideanClusterExtractor.setMaxClusterSize (this->maxClusterSize);
//	euclideanClusterExtractor.setSearchMethod (tree);
//	euclideanClusterExtractor.setInputCloud(inCloudPclPtr);
//	euclideanClusterExtractor.extract (cluster_indices);
//
//	//printf("Number of objects found: %d", cluster_indices.size());
//
//	//Building up the pointclouds for corresponding clusters
//	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
//		brics_3d::ColoredPointCloud3D tempPointCloud3D;
//		tempPointCloud3D.getPointCloud()->clear();
//		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
//
//			int rgbVal= *reinterpret_cast<int*>(&inCloudPclPtr->points[*pit].rgb);
//			uint8_t r, g, b;
//			colorSpaceConvertor.rgb24bitToRGB(rgbVal, &r, &g, &b);
//			unsigned char red= *reinterpret_cast<unsigned char*>(&r);
//			unsigned char green= *reinterpret_cast<unsigned char*>(&g);
//			unsigned char blue= *reinterpret_cast<unsigned char*>(&b);
//	 		tempPointCloud3D.addPoint(new ColoredPoint3D(new Point3D(inCloudPclPtr->points[*pit].x,
//					inCloudPclPtr->points[*pit].y, inCloudPclPtr->points[*pit].z),
//					red, green, blue));
//
//		}
//		extractedClusters->push_back(&tempPointCloud3D);
//	}
//}
}
}
