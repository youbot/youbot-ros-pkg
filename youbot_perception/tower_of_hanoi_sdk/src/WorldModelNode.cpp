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

//ROS specific headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tower_of_hanoi_sdk/GetSceneObjects.h>

//BRICS_3D specific headers
#include "worldModel/WorldModel.h"
#include "worldModel/sceneGraph/Box.h"
#include "core/HomogeneousMatrix44.h"
#include "core/Logger.h"

//System headers
#include <sstream>

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

		startFrameId = "start";
//		auxiliaryFrameId = "auxiliary";
		auxiliaryFrameId = "auxillary";
		goalFrameId = "goal";
		redBoxFrameId = "red_object_";
		greenBoxFrameId = "green_object_";
		yellowBoxFrameId = "yellow_object_";


		objectClasses.clear();
		vector<BRICS_3D::RSG::Attribute> objectAttributes;

		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("taskType","targetArea"));
		objectAttributes.push_back(Attribute("name","start"));
		objectClasses.insert(std::make_pair(startFrameId, objectAttributes));

		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("taskType","targetArea"));
		objectAttributes.push_back(Attribute("name","auxiliary"));
		objectClasses.insert(std::make_pair(auxiliaryFrameId, objectAttributes));

		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("taskType","targetArea"));
		objectAttributes.push_back(Attribute("name","goal"));
		objectClasses.insert(std::make_pair(goalFrameId, objectAttributes));


		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("color","red"));
		objectClasses.insert(std::make_pair(redBoxFrameId + "1", objectAttributes));
		objectClasses.insert(std::make_pair(redBoxFrameId + "2", objectAttributes));
		objectClasses.insert(std::make_pair(redBoxFrameId + "3", objectAttributes));

		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("color","green"));
		objectClasses.insert(std::make_pair(greenBoxFrameId + "1", objectAttributes));
		objectClasses.insert(std::make_pair(greenBoxFrameId + "2", objectAttributes));
		objectClasses.insert(std::make_pair(greenBoxFrameId + "3", objectAttributes));

		objectAttributes.clear();
		objectAttributes.push_back(Attribute("shapeType","Box"));
		objectAttributes.push_back(Attribute("color","yellow"));
		objectClasses.insert(std::make_pair(yellowBoxFrameId + "1", objectAttributes));
		objectClasses.insert(std::make_pair(yellowBoxFrameId + "2", objectAttributes));
		objectClasses.insert(std::make_pair(yellowBoxFrameId + "3", objectAttributes));

	}

	bool onGetSceneObjects (tower_of_hanoi_sdk::GetSceneObjects::Request &req,
	           tower_of_hanoi_sdk::GetSceneObjects::Response &res) {

		/* parse request */
		ROS_DEBUG("Receiving new query.");
		vector<BRICS_3D::RSG::Attribute> queryAttributes;
		for (unsigned int i = 0; i < static_cast<unsigned int>(req.attributes.size()); ++i) {
			queryAttributes.push_back(Attribute(req.attributes[i].key ,req.attributes[i].value));
		}

		/* query */
		vector<BRICS_3D::SceneObject> resultObjects;
		myWM.getSceneObjects(queryAttributes, resultObjects);

		/* setup response */
		res.results.resize(resultObjects.size());
		geometry_msgs::Quaternion tmpQuaternion;
		geometry_msgs::TransformStamped tmpTransformMsg;
		std::stringstream objectSceneFrameID;
		for (unsigned int i = 0; i < static_cast<unsigned int>(resultObjects.size()); ++i) {
			tower_of_hanoi_sdk::SceneObject tmpSceneObject;
			tmpSceneObject.id = resultObjects[i].id;
			tmpSceneObject.parentId = resultObjects[i].parentId;

			tf::Transform tmpTransform;
			homogeniousMatrixToTfTransform(resultObjects[i].transform, tmpTransform);
			tmpTransformMsg.header.stamp = ros::Time::now();
			tmpTransformMsg.header.frame_id = rootFrameId;
			objectSceneFrameID.str("");
			objectSceneFrameID << "scene_object_" << resultObjects[i].id;
			tmpTransformMsg.child_frame_id = objectSceneFrameID.str();
			tmpTransformMsg.transform.translation.x = tmpTransform.getOrigin().getX();
			tmpTransformMsg.transform.translation.y = tmpTransform.getOrigin().getY();
			tmpTransformMsg.transform.translation.z = tmpTransform.getOrigin().getZ();
			tmpTransformMsg.transform.rotation.x = tmpTransform.getRotation().getX();
			tmpTransformMsg.transform.rotation.y = tmpTransform.getRotation().getY();
			tmpTransformMsg.transform.rotation.z = tmpTransform.getRotation().getZ();
			tmpTransformMsg.transform.rotation.w = tmpTransform.getRotation().getW();

			tmpSceneObject.transform = tmpTransformMsg;

			BRICS_3D::RSG::Box::BoxPtr tmpBox = boost::dynamic_pointer_cast<Box>(resultObjects[i].shape);

			if ( tmpBox != 0) {
				tmpSceneObject.shape.type = geometric_shapes_msgs::Shape::BOX; //TODO support multible shapes
				tmpSceneObject.shape.dimensions.resize(3);
				tmpSceneObject.shape.dimensions[0] = tmpBox->getSizeX();
				tmpSceneObject.shape.dimensions[1] = tmpBox->getSizeY();
				tmpSceneObject.shape.dimensions[2] = tmpBox->getSizeZ();
			}


			tmpSceneObject.attributes.resize(resultObjects[i].attributes.size());
			for (unsigned int j = 0; j < static_cast<unsigned int>(resultObjects[i].attributes.size()); ++j) {
				tmpSceneObject.attributes[j].key = resultObjects[i].attributes[j].key;
				tmpSceneObject.attributes[j].value = resultObjects[i].attributes[j].value;
			}

			res.results[i] = tmpSceneObject;
		}


		return true;
	}

	void processTfTopic () {
		map <string, vector<BRICS_3D::RSG::Attribute> >::iterator iter = objectClasses.begin();
		for (iter = objectClasses.begin(); iter != objectClasses.end(); iter++) {

			string objectFrameId = iter->first;
			tf::StampedTransform transform;
			try{
				tfListener.lookupTransform(rootFrameId, objectFrameId, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_WARN("%s",ex.what());
				continue;
			}

			ROS_INFO("TF found for %s.", iter->first.c_str());

			/* query */
			vector<BRICS_3D::RSG::Attribute> queryAttributes;;
			queryAttributes = iter->second;
			vector<BRICS_3D::SceneObject> resultObjects;

			myWM.getSceneObjects(queryAttributes, resultObjects);
//			ROS_INFO("Number of boxes: %i " , static_cast<unsigned int>(resultObjects.size()));

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
				BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform(new BRICS_3D::HomogeneousMatrix44());
				tfTransformToHomogeniousMatrix(transform, newTransform);
				myWM.insertTransform(resultObjects[index].id, newTransform);

			} else {

				/* insert */
				ROS_INFO("Inserting new scene object");
				BRICS_3D::RSG::Shape::ShapePtr boxShape(new BRICS_3D::RSG::Box(0.054, 0.054, 0.054)); // in [m]
				BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr initialTransform(new BRICS_3D::HomogeneousMatrix44());
				tfTransformToHomogeniousMatrix(transform, initialTransform);
				BRICS_3D::SceneObject tmpSceneObject;
				tmpSceneObject.shape = boxShape;
				tmpSceneObject.transform = initialTransform;
				tmpSceneObject.parentId =  myWM.getRootNodeId(); // hook in after root node
				tmpSceneObject.attributes.clear();
				tmpSceneObject.attributes = iter->second;

				unsigned int returnedId;
				myWM.addSceneObject(tmpSceneObject, returnedId);
			}

		}

		/* query */
		vector<BRICS_3D::RSG::Attribute> queryAttributes;
		vector<BRICS_3D::SceneObject> resultObjects;
		queryAttributes.push_back(Attribute("shapeType","Box"));
//		queryAttributes.push_back(Attribute("color","red"));

		myWM.getSceneObjects(queryAttributes, resultObjects);
		ROS_INFO("Total number of boxes: %i " , static_cast<unsigned int>(resultObjects.size()));

	}



	/* Some helper functions */
	void tfTransformToHomogeniousMatrix (const tf::Transform& tfTransform, BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
	{
		double mv[12];

		tfTransform.getBasis().getOpenGLSubMatrix(mv);
		tf::Vector3 origin = tfTransform.getOrigin();

		double* matrixPtr = transformMatrix->setRawData();

		/* matrices are column-major */
		matrixPtr[0] = mv[0]; matrixPtr[4] = mv[4]; matrixPtr[8] = mv[8];   matrixPtr[12] = origin.x();
		matrixPtr[1] = mv[1]; matrixPtr[5] = mv[5]; matrixPtr[9] = mv[9];   matrixPtr[13] = origin.y();
		matrixPtr[2] = mv[2]; matrixPtr[6] = mv[6]; matrixPtr[10] = mv[10]; matrixPtr[14] = origin.z();
		matrixPtr[3] = 1;     matrixPtr[7] = 1;     matrixPtr[11] = 1;      matrixPtr[15] = 1;

	}

	void homogeniousMatrixToTfTransform (const BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix, tf::Transform& tfTransform) {
		const double* matrixPtr = transformMatrix->getRawData();

		btVector3 translation;
		btMatrix3x3 rotation;

		translation.setX(matrixPtr[12]);
		translation.setY(matrixPtr[13]);
		translation.setZ(matrixPtr[14]);

		rotation.setValue(
				matrixPtr[0], matrixPtr[4], matrixPtr[8],
				matrixPtr[1], matrixPtr[5], matrixPtr[9],
				matrixPtr[2], matrixPtr[6], matrixPtr[10]
		);

		tfTransform.setOrigin(translation);
		tfTransform.setBasis(rotation);
	}

	/// To this TF frame_id all objects in the world model relate. Default is the Kinect frame.
	string rootFrameId;

	/// Minimal distance the needs to be exceeded to associate a perceived object with an already seen/stored one.
	double associationDistanceTreshold;

	/// The x,y,z sizes of the cubes to be grasped.
	double cubeSize;

private:

	/// The ROS node handle
	ros::NodeHandle node;

	/// Handle for the 3D world model
	BRICS_3D::WorldModel myWM;

	/// Receives TF
	tf::TransformListener tfListener;

	/// Provided service to get access to the stored data
	ros::ServiceServer getObjectsService;

	/* hanoi specifc IDs */
	string startFrameId;
	string auxiliaryFrameId;
	string goalFrameId;

	string redBoxFrameId;
	string greenBoxFrameId;
	string yellowBoxFrameId;

	/// Mapping that assignes attributes to relevant TF frame_ids
	map <string, vector<BRICS_3D::RSG::Attribute> > objectClasses;
};


}  // namespace youBot


int main(int argc, char **argv)
{

	ros::init(argc, argv, "youbot_3d_world_model");
	ros::NodeHandle n;
	youBot::YouBotWorldModel youbotWM(n);


	/* configuration */
	n.param<std::string>("worldModelRootFrameId", youbotWM.rootFrameId, "/openni_rgb_optical_frame");
	n.param<double>("worldModelcubeSize", youbotWM.cubeSize, 0.05);
	n.param<double>("worldModelassociationDistanceTreshold", youbotWM.associationDistanceTreshold, 0.02);

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
