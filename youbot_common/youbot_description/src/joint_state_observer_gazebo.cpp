/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
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
 * * Neither the name of GPS GmbH nor the names of its
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

#include <joint_state_observer_gazebo.h>
#include "youbot_universal_control.h"

namespace controller {

    JointStateObserverGazebo::JointStateObserverGazebo(controller::YouBotUniversalController* controller) : controller(controller) {
    }

    JointStateObserverGazebo::JointStateObserverGazebo(const JointStateObserverGazebo& orig) : controller(controller) {
    }

    JointStateObserverGazebo::~JointStateObserverGazebo() {
    }

    void JointStateObserverGazebo::updatePosition(const brics_actuator::JointPositions& positions) {

        controller->positionCommand(positions);
    }

    void JointStateObserverGazebo::updateVelocity(const brics_actuator::JointVelocities& velocities) {

        controller->velocityCommand(velocities);
    }

    void JointStateObserverGazebo::updateTorque(const brics_actuator::JointTorques& torques) {
        controller->torqueCommand(torques);
    }

}