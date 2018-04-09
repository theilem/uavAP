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
