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
#include <tf/transform_listener.h>
#include <tower_of_hanoi_sdk/GetSceneObjects.h>

//BRICS_3D specific headers
#include "worldModel/WorldModel.h"
#include "worldModel/sceneGraph/Box.h"
#include "core/HomogeneousMatrix44.h"
#include "core/Logger.h"

namespace youBot {

class YouBotWorldModel {
public:

	YouBotWorldModel(ros::NodeHandle n) :
		node(n) {
		initialize();
	}

	virtual ~YouBotWorldModel() {

	}

	void initialize () {
		rootFrameId = "/openni_rgb_optical_frame";
		associationDistanceTreshold = 0.02; // [m]

		getObjectsService = node.advertiseService("youbot_3d_world_model/getSceneObjects", &YouBotWorldModel::onGetSceneObjects, this);
//		BRICS_3D::Logger::setMinLoglevel(BRICS_3D::Logger::LOGDEBUG);
	}

	bool onGetSceneObjects (tower_of_hanoi_sdk::GetSceneObjects::Request &req,
	           tower_of_hanoi_sdk::GetSceneObjects::Response &res) {

		/* parse request */
		ROS_DEBUG("Receiving new query.");
		vector<BRICS_3D::RSG::Attribute> queryArributes;
		for (unsigned int i = 0; i < static_cast<unsigned int>(req.attributes.size()); ++i) {
			queryArributes.push_back(Attribute(req.attributes[i].key ,req.attributes[i].value));
		}

		/* query */
		vector<BRICS_3D::SceneObject> resultObjects;
		myWM.getSceneObjects(queryArributes, resultObjects);

		/* setup response */
		res.results.resize(resultObjects.size());
		geometry_msgs::Quaternion tmpQuaternion;
		geometry_msgs::TransformStamped tmpTransform;
		for (unsigned int i = 0; i < static_cast<unsigned int>(resultObjects.size()); ++i) {
			tower_of_hanoi_sdk::SceneObject tmpSceneObject;
			tmpSceneObject.id = resultObjects[i].id;
			tmpSceneObject.parentId = resultObjects[i].parentId;

			const double* matrixPtr;
			matrixPtr = resultObjects[i].transform->getRawData();
			double xStored = matrixPtr[12];
			double yStored = matrixPtr[13];
			double zStored = matrixPtr[14];

			tmpQuaternion = tf::createQuaternionMsgFromYaw(0.0); //TODO correct rotation
			tmpTransform.header.stamp = ros::Time::now();
			tmpTransform.header.frame_id = rootFrameId;
			tmpTransform.child_frame_id = "object_X"; //TODO node ID
			tmpTransform.transform.translation.x = xStored;
			tmpTransform.transform.translation.y = yStored;
			tmpTransform.transform.translation.z = zStored;
			tmpTransform.transform.rotation = tmpQuaternion;

			tmpSceneObject.transform = tmpTransform;

			res.results[i] = tmpSceneObject;
		}


		return true;
	}

	void processTfTopic () {
		string objectFrameId = "red_object_1";
		tf::StampedTransform transform;
		try{
			tfListener.lookupTransform(rootFrameId, objectFrameId, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_WARN("%s",ex.what());
		}
		ROS_INFO("TF found.");


		/* query */
		vector<BRICS_3D::RSG::Attribute> queryArributes;
		queryArributes.push_back(Attribute("shapeType","Box"));
		queryArributes.push_back(Attribute("color","red"));
		vector<BRICS_3D::SceneObject> resultObjects;

		myWM.getSceneObjects(queryArributes, resultObjects);
		ROS_INFO("Number of boxes: %i " , static_cast<unsigned int>(resultObjects.size()));

		/* associate */
		unsigned int index = -1;
		double minSquardDistanceToExistingObjects = std::numeric_limits<double>::max();
		const double* matrixPtr;

		for (unsigned int i = 0; i < static_cast<unsigned int>(resultObjects.size()) ; ++i) {
			matrixPtr = resultObjects[i].transform->getRawData();
			double squardDistanceToExistingObjects;
			double xPercieved = transform.getOrigin().x();
			double yPercieved = transform.getOrigin().y();
			double zPercieved = transform.getOrigin().z();
			double xStored = matrixPtr[12];
			double yStored = matrixPtr[13];
			double zStored = matrixPtr[14];

			squardDistanceToExistingObjects = 	(xPercieved - xStored) * (xPercieved - xStored) +
												(yPercieved - yStored) * (yPercieved - yStored) +
												(zPercieved - zStored) * (zPercieved - zStored);

			if (squardDistanceToExistingObjects < minSquardDistanceToExistingObjects) {
				minSquardDistanceToExistingObjects = squardDistanceToExistingObjects;
				index = i;
			}
		}

		ROS_INFO("Shortest distance %lf to found result object %i.", minSquardDistanceToExistingObjects, index);

		if (minSquardDistanceToExistingObjects < associationDistanceTreshold) {

			/* update existing */
			ROS_INFO("Updating existing scene object with object ID: %i", resultObjects[index].id);
			BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform(new BRICS_3D::HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
			                                                             0,1,0,
			                                                             0,0,1,
			                                                             transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z())); 						//Translation coefficients
			myWM.insertTransform(resultObjects[index].id, newTransform);

		} else {

			/* insert */
			ROS_INFO("Inserting new scene object");
			BRICS_3D::RSG::Shape::ShapePtr boxShape(new BRICS_3D::RSG::Box(0.054, 0.054, 0.054)); // in [m]
			BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr initialTransform(new BRICS_3D::HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
			                                                             0,1,0,
			                                                             0,0,1,
			                                                             transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z())); 						//Translation coefficients
			BRICS_3D::SceneObject tmpSceneObject;
			tmpSceneObject.shape = boxShape;
			tmpSceneObject.transform = initialTransform;
			tmpSceneObject.parentId =  myWM.getRootNodeId(); // hook in after root node
			tmpSceneObject.attributes.clear();
			tmpSceneObject.attributes.push_back(Attribute("shapeType","Box"));
			tmpSceneObject.attributes.push_back(Attribute("color","red"));

			unsigned int returnedId;
			myWM.addSceneObject(tmpSceneObject, returnedId);
		}

		/* query */
		queryArributes.clear();
		queryArributes.push_back(Attribute("shapeType","Box"));
		queryArributes.push_back(Attribute("color","red"));

		myWM.getSceneObjects(queryArributes, resultObjects);
		ROS_INFO("Number of boxes: %i " , static_cast<unsigned int>(resultObjects.size()));

	}

private:

	/// The ROS node handle
	ros::NodeHandle node;

	/// Handle for the 3D world model
	BRICS_3D::WorldModel myWM;

	/// Receives TF
	tf::TransformListener tfListener;

	ros::ServiceServer getObjectsService;

	string rootFrameId;

	double associationDistanceTreshold;

};


}  // namespace youBot


int main(int argc, char **argv)
{

	ros::init(argc, argv, "youbot_3d_world_model");
	ros::NodeHandle n;
	youBot::YouBotWorldModel youbotWM(n);

	/* coordination */
//	ros::spin();

	ros::Rate rate(10); // (in Hz)
	while (n.ok()){
		ros::spinOnce();
		youbotWM.processTfTopic();
		rate.sleep();
	}

	return 0;
}

/* EOF */
