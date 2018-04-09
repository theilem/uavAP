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
 * SensorData.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: mircot
 */
#include "uavAP/Core/SensorData.h"

Eigen::Vector3d
vectorFloatToDouble(const Eigen::Vector3f& vec)
{
    return vec.cast<double>();
}

Eigen::Vector3f
vectorDoubleToFloat(const Eigen::Vector3d& vec)
{
    return vec.cast<float>();
}

SensorData
fromSensorDataLight(const SensorDataLight& sd)
{
    SensorData data;
    data.position = vectorFloatToDouble(sd.position);
    data.velocity = vectorFloatToDouble(sd.velocity);
    data.acceleration = vectorFloatToDouble(sd.acceleration);
    data.attitude = vectorFloatToDouble(sd.attitude);
    data.angularRate = vectorFloatToDouble(sd.angularRate);
    data.angularAcc = Vector3(0, 0, 0); //Never set upto now
    data.timestamp = sd.timestamp;
    data.velocityAir = static_cast<double>(sd.velocityAir);
    data.velocityGround = static_cast<double>(sd.velocityGround);
    data.angleOfAttack = static_cast<double>(sd.angleOfAttack);
    data.propulsionPower = static_cast<double>(sd.propulsionPower);
    data.consumedEnergy = static_cast<double>(sd.consumedEnergy);
    data.hasGPSFix = sd.flags & 0x1;
    data.autopilotActive = sd.flags & 0x2;
    data.sequenceNr = sd.sequenceNr;

    return data;
}

SensorDataLight
fromSensorData(const SensorData& sd)
{
    SensorDataLight data;
    data.position = vectorDoubleToFloat(sd.position);
    data.velocity = vectorDoubleToFloat(sd.velocity);
    data.acceleration = vectorDoubleToFloat(sd.acceleration);
    data.attitude = vectorDoubleToFloat(sd.attitude);
    data.angularRate = vectorDoubleToFloat(sd.angularRate);
    data.timestamp = sd.timestamp;
    data.velocityAir = static_cast<float>(sd.velocityAir);
    data.velocityGround = static_cast<float>(sd.velocityGround);
    data.angleOfAttack = static_cast<float>(sd.angleOfAttack);
    data.propulsionPower = static_cast<float>(sd.propulsionPower);
    data.consumedEnergy = static_cast<float>(sd.consumedEnergy);
    data.flags = 0x0;
    data.flags |= sd.hasGPSFix ? 0x1 : 0x0;
    data.flags |= sd.autopilotActive ? 0x2 : 0x0;

    data.sequenceNr = sd.sequenceNr;

    return data;
}
