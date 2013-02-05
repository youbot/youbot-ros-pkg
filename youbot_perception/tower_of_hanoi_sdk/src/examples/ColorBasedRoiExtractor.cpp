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

#include "ColorBasedRoiExtractor.h"

namespace brics_3d {

ColorBasedRoiExtractor::ColorBasedRoiExtractor() {}

    ros::Publisher* ColorBasedRoiExtractor::getExtractedRegionPublisher() const
    {
        return extractedRegionPublisher;
    }

    void ColorBasedRoiExtractor::setExtractedRegionPublisher(ros::Publisher *extractedRegionPublisher)
    {
        this->extractedRegionPublisher = extractedRegionPublisher;
    }

ColorBasedRoiExtractor::~ColorBasedRoiExtractor() {
	// TODO Auto-generated destructor stub
}


void ColorBasedRoiExtractor::initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS){
	this->hsvBasedRoiExtractor.setMinH(minLimitH);
	this->hsvBasedRoiExtractor.setMaxH(maxLimitH);
	this->hsvBasedRoiExtractor.setMinS(minLimitS);
	this->hsvBasedRoiExtractor.setMaxS(maxLimitS);
}

void ColorBasedRoiExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

//	ROS_INFO("\ntransferred kinect_raw message successfully...... :)");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsv_extracted_roi_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());


    brics_3d::PointCloud3D *in_cloud = new brics_3d::PointCloud3D();
    brics_3d::PointCloud3D *extracted_cloud = new brics_3d::PointCloud3D();


    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
    pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

    // cast PCL to brics_3d type
    pclTypeCaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
    ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());

	//perform HSV color based extraction
	hsvBasedRoiExtractor.filter(in_cloud, extracted_cloud);
	ROS_INFO("Size of extracted cloud : %d ", extracted_cloud->getSize());

	//convert back to PCl format for publishing
	//pclTypeCaster.convertToPCLDataType(hsv_extracted_roi_ptr, &extracted_cloud);
	pclTypeCaster.convertToPCLDataType(hsv_extracted_roi_ptr, extracted_cloud);

	//setup frame_id of extracted cloud for publishing
	hsv_extracted_roi_ptr->header.frame_id = "/openni_rgb_optical_frame";

	//publish extracted region
	extractedRegionPublisher->publish(*hsv_extracted_roi_ptr);

		delete in_cloud;
		delete extracted_cloud;

}

}

