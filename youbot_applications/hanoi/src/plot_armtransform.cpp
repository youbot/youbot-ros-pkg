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

int main(int argc, char** argv) {
	ros::init(argc, argv, "hanoi");

	ros::NodeHandle node;

    std::cout << "running the node" << std::endl;



        tf::TransformListener listener;
        tf::StampedTransform transform;
        bool data = false;

        /*while( !data ) {
            data = true;
            try {
                listener.lookupTransform("/arm_link_0","/gripper_palm_link",ros::Time(0),transform);
                std::cout << "collect" << std::endl;
            } catch(tf::TransformException ex) {
                data = false;
                std::cout << "waiting.." << std::endl;
            };
        }

        std::cout << "position" << transform.getOrigin().x()  << std::endl;*/
    ros::Rate rate(1.0);

    while(node.ok()) {
        data = true;
        try {
            listener.lookupTransform("/arm_link_0","/gripper_palm_link",ros::Time(0),transform);
        } catch(tf::TransformException ex) {
                data = false;
                std::cout << "waiting.." << std::endl;
            };

        if(data) {
            std::cout << "Position" << std::endl;
            std::cout << "X: "<< transform.getOrigin().x() << std::endl;
            std::cout << "Y: "<< transform.getOrigin().y() << std::endl;
            std::cout << "Z: "<< transform.getOrigin().z() << std::endl;
        }

        rate.sleep();
    }

    std::cout << "node quitting" << std::endl;

    return 0;
}
