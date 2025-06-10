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
 * Content.h
 *
 *  Created on: Aug 3, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_CONTENT_H_
#define UAVAP_CORE_DATAPRESENTATION_CONTENT_H_

#include <cpsCore/Utilities/EnumMap.hpp>

#include "cpsCore/Utilities/IDC/Header/Hash.h"

/**
 * @brief Defines the Target for forwarding of packets. Only used in uavAP.
 */
enum class Target
{
    INVALID = 0, //!< INVALID
    FLIGHT_ANALYSIS, //!< Forward to FlightAnalysis
    FLIGHT_CONTROL, //!< Forward to FlightControl
    MISSION_CONTROL, //!< Forward to MissionControl
    API, //!< Forward to API
    COMMUNICATION, //!< Forward to Communication
    GROUND_STATION, //!< Forward to GroundStation
    EXTERNAL, //!< Forward to External
    BROADCAST //!< Broadcast to everyone
};

ENUMMAP_INIT(Target, {
             {Target::BROADCAST, "broadcast"},
             {Target::FLIGHT_ANALYSIS, "flight_analysis"},
             {Target::FLIGHT_CONTROL, "flight_control"},
             {Target::MISSION_CONTROL, "mission_control"},
             {Target::COMMUNICATION, "communication"},
             {Target::GROUND_STATION, "ground_station"},
             {Target::EXTERNAL, "external"},
             {Target::API, "api"}});

/**
 * @brief Content definitions.
 */
enum class Content
{
    INVALID, //!< INVALID
    //UAVAP to GS
    SENSOR_DATA, //!< SENSOR_DATA
    SERVO_DATA,
    POWER_DATA,
    MISSION, //!< MISSION
    TRAJECTORY, //!< TRAJECTORY
    PID_STATUS, //!< PID_STATUS
    PID_PARAMS, //!< PID_PARAMS
    SINGLE_PID_PARAMS, //!< PID_PARAMS
    INSPECTING_METRICS, //!< INSPECTING_METRICS
    LINEAR_LOCAL_PLANNER_STATUS, //!< LOCAL_PLANNER_STATUS
    MANEUVER_LOCAL_PLANNER_STATUS,
    SAFETY_BOUNDS, //!< SAFETY_BOUNDS
    CONTROLLER_OUTPUT, //!< CONTROLLER_OUTPUT
    CONTROLLER_OUTPUT_TRIM, //!< CONTROLLER_OUTPUT_TRIM
    CONTROLLER_OUTPUT_OFFSET, //!< CONTROLLER_OUTPUT_OFFSET
    LOCAL_FRAME,
    LOCAL_FRAME_MANAGER_PARAMS,
    OVERRIDE_SAFETY_PARAMS,
    THROTTLE_LIMITER_PARAMS,
    MISSION_LIST,
    MANEUVER_LIST,
    CRITICAL_POINTS, //!< Critical points from geofencing
    MISC_VALUES,
    OVERRIDE_LIST,
    MEMBER_DATA, //!< Content used for all member data. The member type is defined in the following string header.

    //GS to FlightControl in UAVAP
    TUNE_PID, //!< TUNE_PID
    TUNE_PITCH_CONSTRAINT, //!< TUNE_PITCH_CONSTRAINT
    TUNE_ROLL_CONSTRAINT, //!< TUNE_ROLL_CONSTRAINT
    TUNE_LINEAR_LOCAL_PLANNER, //!< TUNE_LOCAL_PLANNER
    MANEUVER_LOCAL_PLANNER_PARAMS, //!< TUNE_LOCAL_PLANNER
    SPLINE_GLOBAL_PLANNER_PARAMS, //!< SPLINE GLOBAL PLANNER
    REQUEST_SINGLE_PID_PARAMS, //!< REQUEST_SINGLE_PID_PARAMS
    REQUEST_DATA, //!< REQUEST_DATA
    REQUEST_MEMBER, //!< REQUEST_MEMBER
    REQUEST_CONFIG, //!< REQUEST_CONFIG
    ADVANCED_CONTROL,

    //GS to MissionControl
    OVERRIDE, //!< OVERRIDE
    MAINTAIN, //!< MAINTAIN
    SELECT_MANEUVER_SET, //!< SELECT_MANEUVER_SET
    SELECT_MISSION, //!< SELECT_MISSION
    WIND_ANALYSIS_STATUS, //!< WIND_ANALYSIS_STATUS

    //GS to FlightAnalysis
    SELECT_INSPECTING_METRICS, //!< SELECT_INSPECTING_METRICS
    ABORT_MANEUVER,
    WIND_INFO,

    // Other
    EXTERNAL //!< External packets that are handled by external systems, not by uavAP
};

/**
 * @brief Data request options.
 */
enum class DataRequest
{
    INVALID = 0, //!< INVALID
    MISSION, //!< MISSION
    TRAJECTORY, //!< TRAJECTORY
    PID_PARAMS, //!< PID PARAMETERS
    MISSION_LIST, //!< List of possible missions
    SAFETY_BOUNDS, //!< SAFETY_BOUNDS
    LOCAL_FRAME, //!< LOCAL_FRAME
    OVERRIDE_LIST, //!< OVERRIDE_LIST
    MANEUVERS_LIST, //!< MANEUVERS_LIST
    START_LOGGING, //!< START_LOGGING
    STOP_LOGGING //!< STOP_LOGGING
};

enum class Tuning
{
    INVALID, LOCAL_PLANNER, GLOBAL_PLANNER, CONTROLLER
};

ENUMMAP_INIT(Tuning, {{Tuning::LOCAL_PLANNER, "local_planner"}, {Tuning::GLOBAL_PLANNER,
             "global_planner"}, {Tuning::CONTROLLER, "controller"}});

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Content>, ContentType>
getConfigRequest()
{
    return Content::REQUEST_CONFIG;
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, std::string>, ContentType>
getConfigRequest()
{
    return "req_cfg";
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Hash>, ContentType>
getConfigRequest()
{
    return Hash("request_config");
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Content>, ContentType>
getMemberRequest()
{
    return Content::REQUEST_MEMBER;
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, std::string>, ContentType>
getMemberRequest()
{
    return "req_member";
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Hash>, ContentType>
getMemberRequest()
{
    return Hash("request_member");
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Content>, ContentType>
getMemberData()
{
    return Content::MEMBER_DATA;
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, std::string>, ContentType>
getMemberData()
{
    return "member_data";
}

template <typename ContentType>
std::enable_if_t<std::is_same_v<ContentType, Hash>, ContentType>
getMemberData()
{
    return Hash("member_data");
}

template <typename TargetType>
std::enable_if_t<std::is_same_v<TargetType, Target>, TargetType>
getDefaultTarget()
{
    return Target::BROADCAST;
}

template <typename TargetType>
std::enable_if_t<std::is_same_v<TargetType, std::string>, TargetType>
getDefaultTarget()
{
    return "broadcast";
}

template <typename TargetType>
std::enable_if_t<std::is_same_v<TargetType, Hash>, TargetType>
getDefaultTarget()
{
    return Hash("broadcast");
}

#endif /* UAVAP_CORE_DATAPRESENTATION_CONTENT_H_ */
