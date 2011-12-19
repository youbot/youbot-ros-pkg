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

#include <vector>

#ifndef CKINEMATICS_H_
#define CKINEMATICS_H_

class CKinematics {
public:
	CKinematics();
	virtual ~CKinematics();
	bool calcInverseKinematicsA2A3A4(std::vector<double>& position, std::vector< std::vector<double> >& solutionSet);
	bool calcInverseKinematics(std::vector<double>& position, std::vector< std::vector<double> >& solutionSet);
	void setArmLengths(std::vector<double>& armLengths);
	void setLimits(std::vector< std::vector<double> >& jointLimits);
	void setJointCorrection(std::vector<double>& jointCorrection);

private:
	std::vector<double> lengths;
	std::vector< std::vector<double> > limits;
	std::vector<double> corr_angles;

	void normalize(std::vector<double>& vec);
	void offsetCorrection(std::vector< std::vector<double> >& solutionSet);
	bool isReachable(std::vector<double>& jointSet);
};

#endif /* CKINEMATICS_H_ */
