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
/**
 * @file Content.h
 * @brief Definitions of possible data contents of packets.
 * @date Aug 8, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_CONTENT_H_
#define UAVAP_CORE_DATAPRESENTATION_CONTENT_H_

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/CustomSerialization.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BasicSerializationImpl.hpp"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/ControlOverride.h"
#include "uavAP/Core/protobuf/messages/Shapes.pb.h"
#include "uavAP/Core/protobuf/messages/LocalPlanner.pb.h"
#include "uavAP/API/ChannelMixing.h"

/**
 * @brief Defines the Target for forwarding of packets. Only used in uavAP.
 */
enum class Target
{
	INVALID,        //!< INVALID
	FLIGHT_CONTROL, //!< Forward to FlightControl
	MISSION_CONTROL,//!< Forward to MissionControl
	COMMUNICATION,  //!< Forward to Communication
	API             //!< Forward to API
};

/**
 * @brief Content definitions.
 */
enum class Content
{
	INVALID,                //!< INVALID
	//UAVAP to GS
	SENSOR_DATA,            //!< SENSOR_DATA
	SENSOR_DATA_LIGHT,      //!< SENSOR_DATA_LIGHT
	MISSION,                //!< MISSION
	TRAJECTORY,             //!< TRAJECTORY
	PID_STATUS,             //!< PID_STATUS
	LOCAL_PLANNER_STATUS,   //!< LOCAL_PLANNER_STATUS
	SAFETY_BOUNDS,          //!< SAFETY_BOUNDS
	CHANNEL_MIX,            //!< CHANNEL_MIX
	CONTROLLER_OUTPUT,      //!< CONTROLLER_OUTPUT
	CONTROLLER_OUTPUT_LIGHT,//!< CONTROLLER_OUTPUT_LIGHT

	//GS to FlightControl in UAVAP
	TUNE_PID,               //!< TUNE_PID
	TUNE_PITCH_CONSTRAINT,  //!< TUNE_PITCH_CONSTRAINT
	TUNE_ROLL_CONSTRAINT,   //!< TUNE_ROLL_CONSTRAINT
	TUNE_LOCAL_PLANNER,     //!< TUNE_LOCAL_PLANNER
	REQUEST_DATA,           //!< REQUEST_DATA

	//GS to MissionControl
	OVERRIDE_CONTROL,       //!< OVERRIDE_CONTROL
	SELECT_MANEUVER_SET,    //!< SELECT_MANEUVER_SET
	SELECT_MISSION          //!< SELECT_MISSION
};

/**
 * @brief Data request options.
 */
enum class DataRequest
{
	INVALID = 0,  //!< INVALID
	MISSION,      //!< MISSION
	TRAJECTORY,   //!< TRAJECTORY
	SAFETY_BOUNDS,//!< SAFETY_BOUNDS
	START_LOGGING,//!< START_LOGGING
	STOP_LOGGING  //!< STOP_LOGGING
};



/**
 * Defines how to write the case labels in the switch case of the data presentation.
 */
#define MAP(CONTENT, TYPE, FUNC) case Content::CONTENT: {FUNC(TYPE);}

/**
 * Add Mapping for Content and DataType here.
 * Format:
 * 	MAP(<CONTENT_ENUM>, <DATA_TYPE>, FUNC)
 */
#define MAPPING(FUNC)		MAP(SENSOR_DATA, SensorData, FUNC) 												\
							MAP(SENSOR_DATA_LIGHT, SensorDataLight, FUNC) 									\
							MAP(MISSION, Mission, FUNC)														\
							MAP(TRAJECTORY, Trajectory, FUNC)												\
							MAP(PID_STATUS, PIDStati, FUNC)													\
							MAP(LOCAL_PLANNER_STATUS, LocalPlannerStatus, FUNC)								\
							MAP(SAFETY_BOUNDS, Rectangle, FUNC)												\
							MAP(CHANNEL_MIX, ChannelMixing::AirplaneChannel, FUNC)							\
							MAP(CONTROLLER_OUTPUT, ControllerOutput, FUNC)									\
							MAP(CONTROLLER_OUTPUT_LIGHT, ControllerOutputLight, FUNC)						\
							MAP(TUNE_PID, PIDTuning, FUNC)													\
							MAP(TUNE_PITCH_CONSTRAINT, ConstraintParams, FUNC)								\
							MAP(TUNE_ROLL_CONSTRAINT, ConstraintParams, FUNC)								\
							MAP(TUNE_LOCAL_PLANNER, LocalPlannerParams, FUNC) 								\
							MAP(REQUEST_DATA, DataRequest, FUNC)											\
							MAP(OVERRIDE_CONTROL, ControlOverride, FUNC)									\
							MAP(SELECT_MANEUVER_SET, std::string, FUNC)										\
							MAP(SELECT_MISSION, std::string, FUNC)



#endif /* UAVAP_CORE_DATAPRESENTATION_CONTENT_H_ */
