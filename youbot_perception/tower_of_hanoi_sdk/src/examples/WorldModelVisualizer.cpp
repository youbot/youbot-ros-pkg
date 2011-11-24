/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
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

//ROS specific Headers
#include <ros/ros.h>
#include <tower_of_hanoi_sdk/GetSceneObjects.h>
#include <visualization_msgs/Marker.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_model_vizualizer");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tower_of_hanoi_sdk::GetSceneObjects>("youbot_3d_world_model/getSceneObjects");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("world_model_markers", 1);
  ros::Rate r(2); //[Hz]

  while (ros::ok()) {


	  tower_of_hanoi_sdk::GetSceneObjects srv;
	  srv.request.attributes.resize(1);
	  srv.request.attributes[0].key = "shapeType";
	  srv.request.attributes[0].value = "Box";
	  uint32_t shape = visualization_msgs::Marker::CUBE;


	  if (!client.call(srv)) {
		  ROS_ERROR("Failed to call service GetSceneObjects");
		  return 1;
	  }

	  ROS_INFO("Number of found objects %i", srv.response.results.size());
	  for (unsigned int i = 0; i < static_cast<unsigned int>(srv.response.results.size()); ++i) {
		  ROS_INFO("	Object with ID %i has transform: ", srv.response.results[i].id);
		  std::cout << srv.response.results[i];

		  visualization_msgs::Marker marker;

		  marker.header.frame_id = srv.response.results[i].transform.header.frame_id;
		  marker.header.stamp = ros::Time::now();
		  marker.ns = "youbot_3d_world_model";
		  marker.id = srv.response.results[i].id;
		  marker.type = shape;
		  marker.action = visualization_msgs::Marker::ADD;

		  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		  marker.pose.position.x = srv.response.results[i].transform.transform.translation.x;
		  marker.pose.position.y = srv.response.results[i].transform.transform.translation.y;
		  marker.pose.position.z = srv.response.results[i].transform.transform.translation.z;
		  marker.pose.orientation.x = srv.response.results[i].transform.transform.rotation.x;
		  marker.pose.orientation.y = srv.response.results[i].transform.transform.rotation.y;
		  marker.pose.orientation.z = srv.response.results[i].transform.transform.rotation.z;
		  marker.pose.orientation.w = srv.response.results[i].transform.transform.rotation.w;

		  // Set the scale of the marker -- 1x1x1 here means 1m on a side
		  marker.scale.x = srv.response.results[i].shape.dimensions[0];
		  marker.scale.y = srv.response.results[i].shape.dimensions[1];
		  marker.scale.z = srv.response.results[i].shape.dimensions[2];

		  // Set the color -- be sure to set alpha to something non-zero!
		  marker.color.r = 0.5f;
		  marker.color.g = 0.5f;
		  marker.color.b = 1.0f;
		  marker.color.a = 0.8;

		  marker.lifetime = ros::Duration();

		  // Publish the marker
		  marker_pub.publish(marker);

		  r.sleep();

	  }
  }

  return 0;
}


/* EOF */
