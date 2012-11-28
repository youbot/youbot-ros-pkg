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

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <iostream>

#include "armPosition.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "wristShell");

	ros::NodeHandle node;

    std::cout << "running hanoi" << std::endl;

    hanoi::armPosition pos;

    std::cout << "initialized" << std::endl;

    ros::Rate rate(20.0);

    std::vector<double> position;
    position.insert(position.begin(),5,0.0);

    while(node.ok()) {
        double val;
        std::cout << "set wrist:";
        std::cin >> val;
        position[4] = val;
        pos.setArm(position);
        //rate.sleep();
    }

    std::cout << std::endl << "quitting hanoi" << std::endl;

    return 0;
}
