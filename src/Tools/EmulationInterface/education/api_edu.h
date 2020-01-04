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
 * api_edu.h
 *
 *  Created on: Feb 11, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_API_EDU_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_API_EDU_H_

struct SensorDataEdu;
struct ControllerOutputEdu;

/**
 * @brief Initialize the API. Will be called by the EduInterface on startup.
 * @return 0 on success, other on failure
 */
int
api_initialize();

/**
 * @brief Provides sensor data through the api. Is called in 100Hz.
 * @param data: SensorData simplified for educational version.
 * @return 0 on success, other on failure
 */
int
api_sense(const SensorDataEdu& data);

/**
 * @brief Request control output from the api. Is called in 100Hz.
 * @param control: 	Reference to Controller Output simplified for educational version.
 * 					Has to be filled by autopilot implementation.
 * @return 0 on success, other on failure
 */
int
api_actuate(ControllerOutputEdu& control);

/**
 * @brief Terminates the api. Is called by EduInterface on shut down.
 * @return 0 on success, other on failure
 */
int
api_terminate();

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_API_EDU_H_ */
