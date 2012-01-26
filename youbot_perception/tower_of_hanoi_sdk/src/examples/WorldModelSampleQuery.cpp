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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_query_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tower_of_hanoi_sdk::GetSceneObjects>("youbot_3d_world_model/getSceneObjects");
  tower_of_hanoi_sdk::GetSceneObjects srv;

  /*
   * The possible attributes for the tower of hanoi challenge are:
   *
   * ("shapeType","Box") 		-> common attribute for all objects
   * ("taskType","targetArea") 	-> common attribute for the start, the auxiliary or the goal position
   * ("name","start")			-> specific attribute for the start position
   * ("name","auxiliary")		-> specific attribute for the auxiliary position
   * ("name","goal")			-> specific attribute for the goal position
   * ("color","red")			-> common attribute to a red graspable box
   * ("color","green")			-> common attribute to a green graspable box
   * ("color","yellow")			-> common attribute to a yellow graspable box
   *
   * In case you combine multiple attributes in one query the logical conjunction is AND.
   *
   */


  /* Set up a query */
  srv.request.attributes.resize(1);
//  srv.request.attributes[0].key = "color";
//  srv.request.attributes[0].value = "red";
//  srv.request.attributes[0].key = "taskType";
//  srv.request.attributes[0].value = "targetArea";
  srv.request.attributes[0].key = "shapeType";
  srv.request.attributes[0].value = "Box";

  /* Send the query*/
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service GetSceneObjects");
    return 1;
  }

  /* Evaluate the results */
  ROS_INFO("Number of found objects %i", srv.response.results.size());
  for (unsigned int i = 0; i < static_cast<unsigned int>(srv.response.results.size()); ++i) {
	  ROS_INFO("	Object with ID %i has transform: ", srv.response.results[i].id);
//	  std::cout << srv.response.results[i].transform;
	  std::cout << srv.response.results[i];
  }

  return 0;
}


/* EOF */
