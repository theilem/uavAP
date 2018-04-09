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
 * PIDMapping.h
 *
 *  Created on: Aug 13, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_
#include <string>

#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include <map>

enum class AirplanePIDs
{
    VELOCITY = 0,
    CLIMB_ANGLE,
    ROLL,
    PITCH,
    YAW_RATE,
    RUDDER
};

enum class HelicopterPIDs
{
    VELOCITY_X = 0,
    VELOCITY_Y,
    CLIMB_RATE,
    ROLL,
    PITCH,
    YAW_RATE
};

const static std::map<AirplanePIDs, std::string> AirplanePIDBimapLeft =
{
    {AirplanePIDs::VELOCITY, "velocity"},
    {AirplanePIDs::CLIMB_ANGLE, "climb_angle"},
    {AirplanePIDs::ROLL, "roll"},
    {AirplanePIDs::PITCH, "pitch"},
    {AirplanePIDs::YAW_RATE, "yaw_rate"},
    {AirplanePIDs::RUDDER, "rudder"}
};

const static std::map<HelicopterPIDs, std::string> HelicopterPIDBimapLeft =
{
    {HelicopterPIDs::VELOCITY_X, "velocity_x"},
    {HelicopterPIDs::VELOCITY_Y, "velocity_y"},
    {HelicopterPIDs::CLIMB_RATE, "climb_rate"},
    {HelicopterPIDs::ROLL, "roll"},
    {HelicopterPIDs::PITCH, "pitch"},
    {HelicopterPIDs::YAW_RATE, "yaw_rate"}
};

const static std::map<std::string, AirplanePIDs> AirplanePIDBimapRight =
{
    {"velocity", AirplanePIDs::VELOCITY},
    {"climb_angle", AirplanePIDs::CLIMB_ANGLE},
    {"roll", AirplanePIDs::ROLL},
    {"pitch", AirplanePIDs::PITCH},
    {"yaw_rate", AirplanePIDs::YAW_RATE},
    {"rudder", AirplanePIDs::RUDDER}
};

const static std::map<std::string, HelicopterPIDs> HelicopterPIDBimapRight =
{
    {"velocity_x", HelicopterPIDs::VELOCITY_X},
    {"velocity_y", HelicopterPIDs::VELOCITY_Y},
    {"climb_rate", HelicopterPIDs::CLIMB_RATE},
    {"roll", HelicopterPIDs::ROLL},
    {"pitch", HelicopterPIDs::PITCH},
    {"yaw_rate", HelicopterPIDs::YAW_RATE}
};

struct PIDTuning
{
    int pid;
    Control::PID::Parameters params;
};

struct ConstraintParams
{
    double min;
    double max;
};

struct PIDStatus
{
    double target;
    double value;
};

using PIDStati = std::map<int, PIDStatus>;

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_ */
