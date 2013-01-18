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
	ros::init(argc, argv, "gripperShell");

	ros::NodeHandle node;

    std::cout << "running hanoi" << std::endl;

    hanoi::armPosition pos;

    std::cout << "initialized" << std::endl;

    ros::Rate rate(20.0);

    while(node.ok()) {   
        //ros::spinOnce();
        /*std::vector<double> positions = pos.get();

       for( std::vector<double>::iterator it = positions.begin(); it != positions.end(); it++ ) {
            std::cout << " | " << *it;
        }
        std::cout << std::endl;

        std::cout << "output?" << std::endl;*/
        double val;
        std::cout << "set gripper:";
        std::cin >> val;
        pos.setGripper(val);

        rate.sleep();
    }

    std::cout << std::endl << "quitting hanoi" << std::endl;

    return 0;
}
