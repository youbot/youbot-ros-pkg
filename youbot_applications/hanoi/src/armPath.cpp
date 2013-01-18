/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Malte Vesper
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
#include <fstream>

using namespace std;

void waitForEnter()
{
        std::cout << "Bitte druecken Sie die Eingabetaste um fortzufahren..." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::cin.rdbuf()->in_avail());
        std::cin.get();
}

int main(int argc,char** argv)
{
	ros::init( argc,  argv, "armPathFromFile");
	ros::NodeHandle node;

	CArmControl youBot;
	Eigen::Matrix<double, 5, 1> pos;
	double gripper;
	std::ifstream path("./coords.txt");
	std::string lineData;

	if( !path ) {
	    std::cout << "fucked up";
	    waitForEnter();
	}

    std::cout << "Loaded, enter to start..." << std::endl;

	//while(node.ok())
	{
	    while(getline(path, lineData) && node.ok()) {
            waitForEnter();
            youBot.getJointPositions(pos);
            std::stringstream lineStream(lineData);
            lineStream >> pos(0) >> pos(1) >> pos(2) >> pos(3);// >> pos(4);
            lineStream >> gripper;
            std::cout << "Fahre position an..." << pos(4) << std::endl;
            //std::cout << pos << std::endl;

            if ( !youBot.setCartesianPosition(pos) ) cout << "MAIN:Error: couldn't set position!" << endl;
            if ( !youBot.setGripperPosition(gripper/100.0) ) cout << "MAIN:Error: couldn't set gripper!" << endl;
	    }
	    std::cout << "Done with points..." << std::endl;
	}


	return 0;

}

