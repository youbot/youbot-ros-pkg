/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Malte Vesper, Steffen Waeldele
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

#include <math.h>
#include <iostream>
#include <vector>
#include "CKinematics.h"
#include "CArmControl.h"
#include <Eigen/Core>
#include <ros/ros.h>

//#include <tf/transform_listener.h>
//#include "tower_of_hanoi_sdk/GetSceneObjects.h"

using namespace std;

int main(int argc,char** argv)
{
	ros::init( argc,  argv, "demo");
	ros::NodeHandle node;

    /*tf::TransformListener listener;
    tf::StampedTransform transform;
*/
	CArmControl youBot;
	Eigen::Matrix<double, 5, 1> pos;
	double gripper;


	while(node.ok())
	{
		cout << "Eingabe X Y Z Alpha Beta: " << endl;
		cin >> pos(0) >> pos(1) >> pos(2) >> pos(3);
		pos(4) = 0.2;

		cout << "main.cpp: setze pos" << endl;
		if ( !youBot.setCartesianPosition(pos) ) cout << "Error: couldn't set position!" << endl;
		//cout << "main.cpp: setze gripper" << endl;
		//if ( !youBot.setGripperPosition(gripper) ) cout << "Error: couldn't set gripper!" << endl;
	}
	/*
	pos(0) = 0.3*cos(M_PI/2.0);
	pos(1) = 0.3*sin(M_PI/2.0);
	pos(2) = 0.4;
	pos(3) = M_PI/6.0;
	*/


/*
	cout << "main.cpp: setze pos" << endl;
	if ( !youBot.setCartesianPosition(pos) ) cout << "Error: couldn't set position!" << endl;
	cout << "main.cpp: setze gripper" << endl;
	if ( !youBot.setGripperPosition(0.0) ) cout << "Error: couldn't set gripper!" << endl;
*/
/*
	ros::ServiceClient client = node.serviceClient<tower_of_hanoi_sdk::GetSceneObjects>("youbot_3d_world_model/getSceneObjects");
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
	 // srv.request.attributes.resize(1);
	//  srv.request.attributes[0].key = "color";
	//  srv.request.attributes[0].value = "red";
	//  srv.request.attributes[0].key = "taskType";
	//  srv.request.attributes[0].value = "targetArea";
	//  srv.request.attributes[0].key = "shapeType";
	 // srv.request.attributes[0].value = "Box";

	  /* Send the query*/
	  /*
	  if (!client.call(srv)) {
	    ROS_ERROR("Failed to call service GetSceneObjects");
	    return 1;
	  }
*/
	  /* Evaluate the results */
	  /*
	  ROS_INFO("Number of found objects %i", srv.response.results.size());
	  for (unsigned int i = 0; i < static_cast<unsigned int>(srv.response.results.size()); ++i) {
		  ROS_INFO("	Object with ID %i has transform: ", srv.response.results[i].id);
	  std::cout << srv.response.results[i].transform;
		  std::cout << srv.response.results[i];
	  }



	  try
	  {
		  if ( !listener.waitForTransform("/map","/base_footprint", ros::Time::now(), ros::Duration(10)) )
		  {
			  ROS_ERROR("no transformation...");
			  throw new tf::TransformException("No TransformationFrame");
		  }

	      listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
	  }
	  catch(tf::TransformException ex)
	  {
		  ROS_ERROR(ex.what());
	  };

	  double roll, pitch, yaw;
	 	  transform.getBasis().getRPY(roll, pitch, yaw);
	       std::cout << "Position" << std::endl;

	       std::cout << "X: "<< transform.getOrigin().x() << std::endl;
	       std::cout << "Y: "<< transform.getOrigin().y() << std::endl;
	       std::cout << "Z: "<< transform.getOrigin().z() << std::endl;
	       std::cout << "ANGLE: "<< roll << "  " << pitch << "  " << yaw*180.0/M_PI << std::endl;


	  try
	  {
		  if ( !listener.waitForTransform("/base_footprint","/arm_link_0", ros::Time::now(), ros::Duration(10)) )
		  {
			  ROS_ERROR("no transformation...");
			  throw new tf::TransformException("No TransformationFrame");
		  }

	      listener.lookupTransform("/base_footprint","/arm_link_0",ros::Time(0),transform);
	  }
	  catch(tf::TransformException ex)
	  {
		  ROS_ERROR(ex.what());
	  };

	  transform.getBasis().getRPY(roll, pitch, yaw);

      std::cout << "Position" << std::endl;
      std::cout << "X: "<< transform.getOrigin().x() << std::endl;
      std::cout << "Y: "<< transform.getOrigin().y() << std::endl;
      std::cout << "Z: "<< transform.getOrigin().z() << std::endl;
      std::cout << "ANGLE: "<< roll << "  " << pitch << "  " << yaw*180.0/M_PI << std::endl;

*/

	return 0;

}

	/*
	CKinematics kin1;
	std::vector<double> armlengths;
	std::vector<double> pos;
	std::vector< std::vector<double> > solution;
	std::vector<double> corr;

	corr.push_back(155.0/180.0*M_PI);
	corr.push_back(-146.0/180.0*M_PI);
	corr.push_back(102.5/180.0*M_PI);

	armlengths.push_back(0.155);
	armlengths.push_back(0.135);
	armlengths.push_back(0.218);

	pos.push_back(0.3*cos(M_PI/4.0));
	pos.push_back(0.3*sin(M_PI/4.0));
	pos.push_back(0.4);
	pos.push_back(M_PI/6.0);

	cout << "X:" << pos.at(0) << "  Y:" << pos.at(1) << "  Z:" << pos.at(2) << "  ALPHA:" << pos.at(3)*180/M_PI << endl;

	kin1.setArmLengths(armlengths);
	kin1.setJointCorrection(corr);

	if (kin1.calcInverseKinematics(pos, solution))
	{
		for(unsigned int i=0; i<solution.size(); i++)
		{
			cout << "====================Solution " << i +1 << "==================" << endl;
			for(unsigned int j=0; j<solution.at(i).size(); j++) std::cout << solution.at(i).at(j)*180/M_PI << "\t\t";
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "Error: calculation Inverse!" << std::endl;
	}
	*/

