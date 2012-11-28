/*
 * GTModelGeneration.h
 *
 *  Created on: Nov 23, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef GTMODELGENERATION_H_
#define GTMODELGENERATION_H_

#include "util/SimplePointCloudGeneratorCube.h"
#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"
#include "util/PCLTypecaster.h"

//ROS specific Headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

class GTModelGeneration {

	BRICS_3D::SimplePointCloudGeneratorCube cubeModelGenerator;



public:

	double cubeLength;
	BRICS_3D::PointCloud3D *cubeModel;

	GTModelGeneration();
	virtual ~GTModelGeneration();
};

#endif /* GTMODELGENERATION_H_ */
