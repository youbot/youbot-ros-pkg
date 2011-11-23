/*
 * GTModelGeneration.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "GTModelGeneration.h"

GTModelGeneration::GTModelGeneration() {

	//Generate the cubemodel
	cubeLength = 0.048;
	cubeModel = new BRICS_3D::PointCloud3D();
	cubeModelGenerator.setPointsOnEachSide(10);
	cubeModelGenerator.setCubeSideLength(cubeLength);
	cubeModelGenerator.setNumOfFaces(3);
	cubeModelGenerator.generatePointCloud(cubeModel);
}

GTModelGeneration::~GTModelGeneration() {
	delete cubeModel;

}

/**
 * Helper function to calculate a homogeneous matrix for affine-transformation
 * @param xRot		Rotation along x-axis
 * @param yRot		Rotation along y-axis
 * @param zRot		Rotation along z-axis
 * @param xtrans	Translation along x-axis
 * @param ytrans	Translation along y-axis
 * @param ztrans	Translation along z-axis
 * @param homogeneousMatrix	Output matrix
 * @param inDegrees	Indicates whether the rotation angles are in degree(1) or radian(0)
 */
void calculateHomogeneousMatrix(float xRot,float yRot,
		float zRot, float xtrans, float ytrans, float ztrans, Eigen::Matrix4f  &homogeneousMatrix, bool inDegrees){
	if(inDegrees){
		float PI = 3.14159265;
		xRot = xRot*(PI/180);
		yRot *= yRot*(PI/180);
		zRot *= zRot*(PI/180);
	}
	/*
	 * layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
//	std::cout <<"Translation: " << xtrans << " " << ytrans <<" " << ztrans << std::endl;

	homogeneousMatrix(3)=0;
	homogeneousMatrix(7)=0;
	homogeneousMatrix(11)=0;
	homogeneousMatrix(15)=1;

    //translation
    homogeneousMatrix(12)=xtrans;
	homogeneousMatrix(13)=ytrans;
	homogeneousMatrix(14)=ztrans;

	//rotation
	homogeneousMatrix(0) = cos(yRot)*cos(zRot);
	homogeneousMatrix(1) = cos(yRot)*sin(zRot);
	homogeneousMatrix(2) = -sin(yRot);

	homogeneousMatrix(4) = -cos(xRot)*sin(zRot) + sin(xRot)*sin(yRot)*cos(zRot);
	homogeneousMatrix(5) = cos(xRot)*cos(zRot) + sin(xRot)*sin(yRot)*sin(zRot);
	homogeneousMatrix(6) = sin(xRot)*cos(yRot);

	homogeneousMatrix(8) = sin(xRot)*sin(zRot) + cos(xRot)*sin(yRot)*cos(zRot);
	homogeneousMatrix(9) = -sin(xRot)*cos(zRot) + cos(xRot)*sin(yRot)*sin(zRot);
	homogeneousMatrix(10)= cos(xRot)*cos(yRot);


}


void applyHomogeneousTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr
transformedCloudCube, Eigen::Matrix4f  &homogenousMatrix){
	transformedCloudCube->width = cloud->width;
	transformedCloudCube->height = cloud->height;
	transformedCloudCube->is_dense = false;
	transformedCloudCube->points.resize (transformedCloudCube->width * transformedCloudCube->height);
	/*
	 * layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	/* rotate */

	for (unsigned int i = 0; i < cloud->size(); i++ ) {
		float xTemp,yTemp,zTemp;
		xTemp = (cloud->points[i].x * homogenousMatrix[0] + cloud->points[i].y * homogenousMatrix[4] +
				cloud->points[i].z * homogenousMatrix[8]);
		yTemp = cloud->points[i].x * homogenousMatrix[1] + cloud->points[i].y * homogenousMatrix[5] +
				cloud->points[i].z * homogenousMatrix[9];
		zTemp = cloud->points[i].x * homogenousMatrix[2] + cloud->points[i].y * homogenousMatrix[6] +
				cloud->points[i].z * homogenousMatrix[10];

		/* translate */
		transformedCloudCube->points[i].x = (xTemp + homogenousMatrix[12]);
		transformedCloudCube->points[i].y = (yTemp + homogenousMatrix[13]);
		transformedCloudCube->points[i].z = (zTemp + homogenousMatrix[14]);
	}



}

int main(int argc, char* argv[]){

	ros::init(argc, argv, "GTModelGenerator");
	ros::NodeHandle nh;

	tf::StampedTransform transformPatternBase;
	GTModelGeneration *gtModel = new GTModelGeneration();
	tf::TransformListener listener;
	/**
	 * Object to typecast data-types between PCL and BRICS_3D
	 */
	BRICS_3D::PCLTypecaster pclTypecaster;
	/**
	 * Publisher to output the fitted models
	 */
	ros::Publisher modelPublisher = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("GT_Model", 1);
	ros::Rate rate(10.0);

	static tf::TransformBroadcaster br;
		     tf::Transform transform;

	while (nh.ok()){

		//Get the transform for pattern base
		try{
			listener.lookupTransform("/openni_rgb_optical_frame","/PATTERN_BASE",

					ros::Time(0), transformPatternBase);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}

		//Extract the origin and the euler angles from the received transform
		tf::Vector3 origin = transformPatternBase.getOrigin();
		if(!std::isnan(origin [0]) &&  !std::isnan(origin [1]) && !std::isnan(origin [2])){
//			std::cout <<"Translation: " << origin [0] << " " << origin [1] <<" " << origin [2] << std::endl;

		} else {
			continue;
		}

		tf::Quaternion rotation = transformPatternBase.getRotation();

		btScalar roll, pitch, yaw;
		unsigned int solution_number=1;
		btMatrix3x3(rotation).getEulerZYX(yaw, pitch, roll,solution_number);
		if(!std::isnan(yaw) &&  !std::isnan(pitch) && !std::isnan(roll)){
//			std::cout << "Rotation: " <<yaw << " " <<pitch << " " <<roll << " " <<std::endl;

		}else{
		continue;
		}


//	     transform.setOrigin( tf::Vector3(origin[0],origin[1],origin[2]+0.024) );
//	     //Todo stop using Quaternion
//	     transform.setRotation( tf::Quaternion(roll, pitch, yaw) );
//	     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/PATTERN_BASE","gt_frame"));

		//-------------------------------------------------Transforming the model

//		origin[2] -= gtModel->cubeLength;
//		origin[1] -= gtModel->cubeLength;

		Eigen::Matrix4f  tempHomogenousMatrix;
		calculateHomogeneousMatrix(roll,pitch,yaw,origin[0],origin[1],origin[2],tempHomogenousMatrix,false);
//
//		BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new BRICS_3D::HomogeneousMatrix44(
//				1, 0, 0,
//					0, 1, 0,
//					0, 0, 1,
//					0,0,0);
//
//		gtModel->cubeModel->homogeneousTransformation(homogeneousTrans);


		//-------------------------------------------------Publishing the model



		pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_model_ptr(new pcl::PointCloud<pcl::PointXYZ>());
		pclTypecaster.convertToPCLDataType(estimated_model_ptr,gtModel->cubeModel);

		applyHomogeneousTransformation(estimated_model_ptr, estimated_model_ptr, tempHomogenousMatrix);

		estimated_model_ptr->header.frame_id = "/openni_rgb_optical_frame";
		modelPublisher.publish(*estimated_model_ptr);

		}

	delete gtModel;
}
