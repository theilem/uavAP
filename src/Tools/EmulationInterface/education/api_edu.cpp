////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * api_edu.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: mircot
 */

#include "api_edu.h"
#include "SensorDataEdu.h"
#include "ControllerOutputEdu.h"

#include <iostream>

uint32_t sequenceNr = 0;

int
api_initialize()
{
	std::cout << "Initialize called." << std::endl;
	return 0;
}

int
api_sense(const SensorDataEdu& data)
{
	std::cout << "Sensor Data received: " << std::endl;
	std::cout << "Position: " << data.position[0] << ", " << data.position[1] << ", "
			<< data.position[2] << std::endl;
	std::cout << "Velocity: " << data.velocity[0] << ", " << data.velocity[1] << ", "
			<< data.velocity[2] << std::endl;
	std::cout << "Accelera: " << data.acceleration[0] << ", " << data.acceleration[1] << ", "
			<< data.acceleration[2] << std::endl;
	std::cout << "Attitude: " << data.attitude[0] << ", " << data.attitude[1] << ", "
			<< data.attitude[2] << std::endl;
	std::cout << "AngularR: " << data.angularRate[0] << ", " << data.angularRate[1] << ", "
			<< data.angularRate[2] << std::endl;
	std::cout << "====================================================" << std::endl;
	sequenceNr = data.sequenceNr;
	return 0;
}

int
api_actuate(ControllerOutputEdu& control)
{
	std::cout << "Actuate called" << std::endl;
	control.flapOutput = 0;
	control.pitchOutput = 0.1;
	control.rollOutput = 0.1;
	control.throttleOutput = 0.5;
	control.yawOutput = 0;

	control.sequenceNr = sequenceNr;
	return 0;
}

int
api_terminate()
{
	std::cout << "Terminate called." << std::endl;
	return 0;
}
