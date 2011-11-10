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

#ifndef COLORBASEDROIEXTRACTOR_H_
#define COLORBASEDROIEXTRACTOR_H_

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"

#include "pcl/filters/passthrough.h"
#include "pcl/io/pcd_io.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"

#include "algorithm/filtering/ColorBasedROIExtractorHSV.h"
#include "core/ColorSpaceConvertor.h"
#include "util/PCLTypecaster.h"
#include "core/ColoredPointCloud3D.h"

namespace BRICS_3D {

//ToDo update the extractor to also use some default V values to be more flexible.

class ColorBasedRoiExtractor {
private:

	/**
	 * ROS publisher to publish the extracted Regions of Interests
	 */
	ros::Publisher *extractedRegionPublisher;

	/**
	 * object for extracting ROIs based on HSV-color space limits
	 */
	BRICS_3D::ColorBasedROIExtractorHSV hsvBasedRoiExtractor;

	/**
	 * Utility object for color-space transforms
	 */
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;

	/**
	 * Utility object for type-casting data between BRICS_3D and PCL
	 */
	BRICS_3D::PCLTypecaster pclTypeCaster;

public:
	ColorBasedRoiExtractor();
	virtual ~ColorBasedRoiExtractor();

	/**
	 * Callback function for Kinect data. Finds the ROI from the input data
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


	/**
	 *
	 * @return the current publisher which will be used to publish the extracted regions of interests
	 */
	ros::Publisher* getExtractedRegionPublisher() const;


	/**
	 * sets the current publisher which will be used to publish the extracted regions of interests
	 * @param extractedRegionPublisher the current publisher which will be used to publish the extracted ROIs
	 */
    void setExtractedRegionPublisher(ros::Publisher *extractedRegionPublisher);



};

}

#endif /* COLORBASEDROIEXTRACTOR_H_ */
