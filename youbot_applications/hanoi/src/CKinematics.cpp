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

#include <math.h>
#include <iostream>
#include "CKinematics.h"
#include <Eigen/Core>

CKinematics::CKinematics() {
	// TODO Auto-generated constructor stub
}

CKinematics::~CKinematics() {
	// TODO Auto-generated destructor stub
}

bool CKinematics::calcInverseKinematicsA2A3A4(std::vector<double>& position, std::vector< std::vector<double> >& solutionSet)
{
	double alpha, beta, gamma, theta, temp_length;
	std::vector<double> m;
	std::vector<double> temp_point;

	//falsche Parameteranzahl
	if (position.size() != 3)
	{
		std::cout << "Error: position must contain three elements!" << std::endl;
		return false;
	}

	//temp_point berechnen
	temp_point.push_back(position.at(0)-cos(position.at(2))*lengths.at(2));

		//std::cout << "temp_x=" << temp_point.back() << std::endl;

	temp_point.push_back(position.at(1)-sin(position.at(2))*lengths.at(2));

		//std::cout << "temp_y=" << temp_point.back() << std::endl;

	temp_length = sqrt(pow(temp_point.at(0),2)+pow(temp_point.at(1),2));

		//std::cout << "temp_length=" << temp_length << std::endl;

	//Punkt außerhalb der Reichweite
	if (temp_length > (lengths.at(0) + lengths.at(1)))
	{
		//std::cout << "Error: position out of reach!" << std::endl;
		return false;
	}

	//Winkel berechnen
	theta = atan2(temp_point.at(1), temp_point.at(0));

	//std::cout << "theta=" << theta*180.0/M_PI << std::endl;

	alpha = acos((pow(lengths.at(0),2) + pow(temp_length,2) - pow(lengths.at(1),2)) / (2 * lengths.at(0) * temp_length));

	//std::cout << "alpha=" << alpha*180.0/M_PI << std::endl;

	beta  = acos((pow(lengths.at(0),2) + pow(lengths.at(1),2) - pow(temp_length,2)) / (2 * lengths.at(0) * lengths.at(1)));

	//std::cout << "beta=" << beta*180.0/M_PI << std::endl;

	gamma = acos((pow(lengths.at(1),2) + pow(temp_length,2) - pow(lengths.at(0),2)) / (2 * lengths.at(1) * temp_length));

	//std::cout << "gamma=" << gamma*180.0/M_PI << std::endl;

	//Lösungen generieren
	solutionSet.push_back(std::vector<double>());
	solutionSet.back().push_back(theta + alpha);
	solutionSet.back().push_back(beta - M_PI);
	solutionSet.back().push_back(position.at(2) + gamma - theta);

	solutionSet.push_back(std::vector<double>());
	solutionSet.back().push_back(theta - alpha);
	solutionSet.back().push_back(M_PI - beta);
	solutionSet.back().push_back(position.at(2) - gamma - theta);

	/*
	for(unsigned int i=0; i<solutionSet.size(); i++)
	{
		std::cout << "unmapped:";
		for(unsigned int j=0; j<solutionSet.at(i).size(); j++) std::cout << solutionSet.at(i).at(j)*180/M_PI << "\t";
		std::cout << std::endl;
	}
	*/

	//offsetCorrection
	offsetCorrection(solutionSet);

	return true;
}

void CKinematics::setArmLengths(std::vector<double>& armLengths)
{
	lengths = armLengths;
}

void CKinematics::normalize(std::vector<double>& vec)
{
	double sum=0;

	for(int i=0; i<vec.size(); i++) sum+=pow(vec.at(i),2);

	sum=sqrt(sum);

	for(int i=0; i<vec.size(); i++) vec.at(i)/=sum;
}

void CKinematics::setLimits(std::vector< std::vector<double> >& jointLimits)
{
	limits = jointLimits;
}

void CKinematics::setJointCorrection(std::vector<double>& jointCorrection)
{
	corr_angles = jointCorrection;
}

void CKinematics::offsetCorrection(std::vector< std::vector<double> >& solutionSet)
{
	for(unsigned int i=0; i<solutionSet.size(); i++)
	{
		for(unsigned int j=0; j<solutionSet.at(i).size(); j++)
		{
			solutionSet.at(i).at(j) = corr_angles.at(j)-solutionSet.at(i).at(j);
		}
	}
}


bool CKinematics::calcInverseKinematics(std::vector<double>& position, std::vector< std::vector<double> >& solutionSet)
{
	//position: (x,y,z,alpha)
	double theta;
	int i=0;
	bool failure=false;
	std::vector<double> temp_position = position;
	std::vector<double> pos;
	std::vector< std::vector<double> > solution, solution2;

	temp_position.at(0) = position.at(0) - 0.024;
	temp_position.at(2) = position.at(2) - 0.115;

	theta = atan2(temp_position.at(1), temp_position.at(0));

	//std::cout << "A1_theta:" << theta << std::endl;

	pos.push_back(cos(theta) * temp_position.at(0) + sin(theta) * temp_position.at(1));

	//std::cout << pos.back() << std::endl;

	//std::cout << (- sin(theta) * temp_position.at(0) + cos(theta) * temp_position.at(1)) << std::endl;

	pos.push_back(temp_position.at(2));
	pos.push_back(temp_position.at(3));

	//std::cout << "FALL 1" << std::endl;

	theta = - theta + 169.0/180.0*M_PI;

	//Fall 1
	pos.at(0) -= 0.033;
	if (calcInverseKinematicsA2A3A4(pos, solution))
	{
		bool reachable=true;

		//std::cout << "A1=" << theta << std::endl;

		solutionSet.push_back(std::vector<double>());
		solutionSet.back().push_back(theta);
		solutionSet.back().push_back(solution.at(0).at(0));
		solutionSet.back().push_back(solution.at(0).at(1));
		solutionSet.back().push_back(solution.at(0).at(2));
		solutionSet.back().push_back(position.at(4));

		reachable=isReachable(solutionSet.back());
		if (!reachable) solutionSet.erase(solutionSet.end()-1);

		solutionSet.push_back(std::vector<double>());
		solutionSet.back().push_back(theta);
		solutionSet.back().push_back(solution.at(1).at(0));
		solutionSet.back().push_back(solution.at(1).at(1));
		solutionSet.back().push_back(solution.at(1).at(2));
		solutionSet.back().push_back(position.at(4));

		if (!reachable && !isReachable(solutionSet.back()))
		{
			solutionSet.erase(solutionSet.end()-1);
			failure=true;
		}
	}
	else
	{
		failure=true;
	}

	/*
	for(unsigned int j=0; j<pos.size(); j++) std::cout << pos.at(j)*180/M_PI << "\t";
	std::cout << std::endl;
	*/

	//std::cout << "FALL 2" << std::endl;

	//Fall 2
	pos.at(0) += 2 * 0.033;
	pos.at(0) *= -1.0;
	pos.at(2) = M_PI - pos.at(2);

	i=0;

	if (calcInverseKinematicsA2A3A4(pos, solution2))
	{
		bool reachable=true;

		//std::cout << "A1=" << M_PI - theta << std::endl;

		solutionSet.push_back(std::vector<double>());
		solutionSet.back().push_back(M_PI + theta);
		solutionSet.back().push_back(solution2.at(0).at(0));
		solutionSet.back().push_back(solution2.at(0).at(1));
		solutionSet.back().push_back(solution2.at(0).at(2));
		solutionSet.back().push_back(position.at(4));

		reachable=isReachable(solutionSet.back());
		if (!reachable) solutionSet.erase(solutionSet.end()-1);

		solutionSet.push_back(std::vector<double>());
		solutionSet.back().push_back(M_PI + theta);
		solutionSet.back().push_back(solution2.at(1).at(0));
		solutionSet.back().push_back(solution2.at(1).at(1));
		solutionSet.back().push_back(solution2.at(1).at(2));
		solutionSet.back().push_back(position.at(4));

		if (!reachable && !isReachable(solutionSet.back()) && failure == true)
		{
			solutionSet.erase(solutionSet.end()-1);
			return false;
		}

	}
	else
	{
		if (failure == true) return false;
	}

	/*
	for(unsigned int j=0; j<pos.size(); j++) std::cout << pos.at(j)*180/M_PI << "\t";
	std::cout << std::endl;
	*/

	//std::cout << "END" << std::endl;


	return true;
}

bool CKinematics::isReachable(std::vector<double>& jointSet)
{
	if (jointSet.at(0) < 0.015 || jointSet.at(0) > 5.84) return false;
	if (jointSet.at(1) < 0.015 || jointSet.at(1) > 2.61) return false;
	if (jointSet.at(2) < -5.02 || jointSet.at(2) > -0.016) return false;
	if (jointSet.at(3) < 0.025 || jointSet.at(3) > 3.42) return false;

	return true;
}
