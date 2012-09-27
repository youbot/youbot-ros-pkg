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

#ifndef POSEESTIMATIONICP_H_
#define POSEESTIMATIONICP_H_


#include "core/PointCloud3D.h"
#include "util/PCLTypecaster.h"
#include <iostream>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/common/transforms.h>


namespace BRICS_3D {

namespace SDK {

class IterativeClosestPoint {
private:

	/**
	 * Object model to be fitted with the target cloud
	 */
	BRICS_3D::PointCloud3D *objectModel;

	/**
	 * Maximum distance threshold for ICP
	 */
	float distance;

	/**
	 * Transformation threshold for ICP
	 */
	float transformationEpsilon;

	/**
	 * Maximum number of iterations to be used for ICP
	 */
	int maxIterations;

	/**
	 * Fitness score of the best fit
	 */
	float fitnessScore;

	/**
	 * Best transformation found
	 */
	Eigen::Matrix4f finalTransformation;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IterativeClosestPoint();
	virtual ~IterativeClosestPoint();

	/**
	 * Estimates the best fit between the object model and the inCloud (input cloud)
	 * @param inCloud	input point-cloud
	 * @param outCloud	best transformed result found of the object model
	 */
	void estimateBestFit(BRICS_3D::PointCloud3D *inCloud, BRICS_3D::PointCloud3D *outCloud);


	/**
	 *	Returns the best transformation found
	 * @return	best transformation
	 */
	Eigen::Matrix4f getFinalTransformation() const
    {
        return finalTransformation;
    }


	/**
	 * Returns the fitness score of the best fit
	 * @return 	fitness score of the best fit
	 */
    float getFitnessScore() const
    {
        return fitnessScore;
    }


    /**
     * Returns the maximum iterations used in ICP
     * @return	maximum iterations
     */
    int getMaxIterations() const
    {
        return maxIterations;
    }


    /**
     * Returns the transformation threshold for ICP
     * @return	Transformation threshold for ICP
     */
    float getTransformationEpsilon() const
    {
        return transformationEpsilon;
    }


    /**
     * Set the maximum iteration for ICP
     * @param maxIterations	maximum iterations for ICP
     */
    void setMaxIterations(int maxIterations)
    {
        this->maxIterations = maxIterations;
    }


    /**
     * Set the transformation Threshold for ICP
     * @param transformationEpsilon	transformation Threshold for ICP
     */
    void setTransformationEpsilon(float transformationEpsilon)
    {
        this->transformationEpsilon = transformationEpsilon;
    }


    /**
     * Returns the used object model for fitting
     * @return	The used object model for fitting
     */
    BRICS_3D::PointCloud3D* getObjectModel() const
    {
        return objectModel;
    }


    /**
     * Sets the object model to be used for fitting
     * @param objectModel	object model to be used for fitting
     */
    void setObjectModel(BRICS_3D::PointCloud3D *objectModel)
    {
        this->objectModel = objectModel;
    }


    /**
     * Returns the maximum distance threshold for ICP
     * @return Maximum distance threshold for ICP
     */
    float getDistance() const
    {
        return distance;
    }


    /**
     * Sets the maximum distance threshold for ICP
     * @param distance maximum distance threshold for ICP
     */
    void setDistance(float distance)
    {
        this->distance = distance;
    }

};

}

}

#endif /* POSEESTIMATIONICP_H_ */
