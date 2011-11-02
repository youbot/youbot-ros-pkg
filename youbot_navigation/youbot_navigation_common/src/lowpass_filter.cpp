/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Alexey Zakharov
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

double T = 0; // time constant in sec
double dt = 0; // time interval in sec

double x_ = 0;	// linear velocity in m/sec
double y_ = 0;  // linear velocity in m/sec
double z_ = 0;  // angular velocity in rad/sec

ros::Publisher twistPublisher;

double lowPassFilter(double x, double y0, double dt, double T) //Low-pass filter : 1/(Tp + 1)
{
   return y0 + (x - y0) * (dt/(dt+T));  // y0 - previous output value, x - input value
}

void twistCallback(const geometry_msgs::Twist& twist){
  
  double x =  twist.linear.x;   // current value of linear velocity
  double y = twist.linear.y;    // current value of linear velocity
  double z = twist.angular.z;   // current value of angular velocity

  x_ = lowPassFilter(x, x_, dt, T);
  y_ = lowPassFilter(y, y_, dt, T);
  z_ = lowPassFilter(z, z_, dt, T);

  geometry_msgs::Twist t;  // publishing filtered signal
  t.linear.x = x_;
  t.linear.y = y_;
  t.angular.z = z_;
  twistPublisher.publish(t);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "lowpass_filter");
  ros::NodeHandle n;

  n.param("T", T, 0.3);  // setting default values in seconds
  n.param("dt", dt, 0.1); 

  ros::Subscriber twistSubscriber = n.subscribe("move_base/cmd_vel", 1000, &twistCallback);   
  twistPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);  

  ros::Rate rate(1.0/dt); //setting update rate in Hz
  while (n.ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

