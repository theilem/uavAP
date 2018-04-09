/*
 * ControllerOutputEdu.h
 *
 *  Created on: Feb 11, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_CONTROLLEROUTPUTEDU_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_CONTROLLEROUTPUTEDU_H_

/**
 * @brief 	Simplified version of the uavAP controller output.
 * 			Contains necessary information to control a standard fixed wing aircraft.
 */
struct ControllerOutputEdu
{
	double rollOutput;		//-1: roll right, 1: roll left
	double pitchOutput;		//-1: pitch down, 1: pitch up
	double yawOutput;		//-1: rudder left, 1: rudder right
	double throttleOutput;	// 0: no throttle, 1: full throttle
	double flapOutput; 		// 0: no flap, 1: full flap

	uint32_t sequenceNr; 	// Control sequence number. Necessary for time tracing.
							// Set to sequence number of sensor data used to calculate control.
};


#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EDUCATION_CONTROLLEROUTPUTEDU_H_ */
