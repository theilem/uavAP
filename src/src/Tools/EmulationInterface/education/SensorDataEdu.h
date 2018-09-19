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
 * SensorDataEdu.h
 *
 *  Created on: Feb 11, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_SENSORDATAEDU_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_SENSORDATAEDU_H_
#include <chrono>

/**
 * @brief 	Simplified version of the uavAP sensor data. Contains necessary information to compute control.
 * 			For a time stamp simply take the current system time (Not a perfect solution in real life, but sufficient
 * 			in the simulation)
 */
struct SensorDataEdu
{
	double position[3]; 	// UTM Frame, [X: North, Y: East, Z: Down]
	double velocity[3]; 	// Earth Frame
	double acceleration[3]; // Body Frame
	double attitude[3]; 	// [X: Roll, Y: Pitch, Z: Yaw]
	double angularRate[3]; 	// [X: Roll, Y: Pitch, Z: Yaw]

	uint32_t sequenceNr; 	// Sensor Data sequence number. Necessary for time tracing.
};

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_SENSORDATAEDU_H_ */
