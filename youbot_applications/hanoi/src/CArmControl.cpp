/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Steffen Waeldele
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

#include "CArmControl.h"
#include <vector>
#include <map>

CArmControl::CArmControl()
{
	corr.push_back(155.0/180.0*M_PI);
	corr.push_back(-146.0/180.0*M_PI);
	corr.push_back(102.5/180.0*M_PI);

	armlengths.push_back(0.155);
	armlengths.push_back(0.135);
	armlengths.push_back(0.218);

	armKin.setArmLengths(armlengths);
	armKin.setJointCorrection(corr);
}

CArmControl::~CArmControl()
{
	// TODO Auto-generated destructor stub
}

void CArmControl::getJointPositions(Eigen::Matrix<double, 5, 1>& currentPosition)
{
	std::vector<double> currPos;

	currPos = armIO.get();
	for (unsigned int i=0; i<5; i++) currentPosition(i, 0) = currPos.at(i);
	std::cout << currentPosition << std::endl;
}

bool CArmControl::setJointPositions(Eigen::Matrix<double, 5, 1>& setPosition)
{
	std::vector<double> setPos;

	for (unsigned int i=0; i<5; i++) setPos.push_back(setPosition(i, 0));
	armIO.setArm(setPos);
	return true;
}

void CArmControl::sortSolutionSet(std::vector< std::vector<double> >& solutionSet, std::vector< Eigen::Matrix<double, 5 ,1> >& sortedSolutionSet)
{
	Eigen::Matrix<double, 5, 1> currPos, tempPos, deltaPos;
	std::map< double, Eigen::Matrix<double, 5, 1> > solutionSetMap;
	std::map< double, Eigen::Matrix<double, 5, 1> >::iterator it;;
	double max;

	getJointPositions(currPos);

	for (unsigned int i=0; i<solutionSet.size(); i++)
	{
		for (unsigned int j=0; j<solutionSet.at(i).size(); j++) tempPos(j, 0) = solutionSet.at(i).at(j);

		deltaPos = currPos - tempPos;
		deltaPos = deltaPos.array().abs();
		max = deltaPos.maxCoeff();

		solutionSetMap.insert( std::pair< double, Eigen::Matrix<double, 5, 1> >( max, tempPos) );


	}

	for(it = solutionSetMap.begin(); it!=solutionSetMap.end(); it++)
	{
		sortedSolutionSet.push_back(it->second);
	}


}

bool CArmControl::setCartesianPosition(Eigen::Matrix<double, 5, 1> cartesianPosition)
{
	//X Y Z ANGLE TCP_ROTATION
	std::vector<double> cartPos;
	std::vector< std::vector<double> > solutionSet;
	std::vector< Eigen::Matrix<double, 5 ,1> > sortedSolutionSet;

	for (unsigned int i=0; i<5; i++) cartPos.push_back(cartesianPosition(i, 0));

	if (!armKin.calcInverseKinematics(cartPos, solutionSet)) return false;
	sortSolutionSet(solutionSet, sortedSolutionSet);
	sortedSolutionSet.at(0)(4) = sortedSolutionSet.at(0)(0)+(165.0-169.0)/180.0*M_PI;
	setJointPositions(sortedSolutionSet.at(0));
	//std::cout << sortedSolutionSet.at(0)(0) << std::endl;

	return true;
}

bool CArmControl::setGripperPosition(double position)
{
	if ( position < 0.0 || position > 0.023 ) return false;

	armIO.setGripper(position);

	return true;
}

void CArmControl::closeGripper()
{
	armIO.setGripper(0.0);
}
