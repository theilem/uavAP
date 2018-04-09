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
