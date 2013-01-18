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

#ifndef CARMCONTROL_H_
#define CARMCONTROL_H_

#include <vector>
#include "CKinematics.h"
#include <Eigen/Core>
#include "armPosition.h"

class CArmControl {
public:
	CArmControl();
	virtual ~CArmControl();

	void getJointPositions(Eigen::Matrix<double, 5, 1>& currentPosition);

	//Format [X Y Z ANGLE TCP_ROTATION]
	bool setCartesianPosition(Eigen::Matrix<double, 5, 1> cartesianPosition);

	//Format [A1, A2, A3, A4, A5]
	bool setJointPositions(Eigen::Matrix<double, 5, 1>& setPosition);

	bool setGripperPosition(double position);
	void closeGripper();


private:
	//Eigen::Matrix<double, 5, 1> setPosition;
	std::vector<double> armlengths;
	std::vector<double> corr;
	CKinematics armKin;
	hanoi::armPosition armIO;

	void sortSolutionSet(std::vector< std::vector<double> >& solutionSet, std::vector< Eigen::Matrix<double, 5 ,1> >& sortedSolutionSet);
};

#endif /* CARMCONTROL_H_ */
