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
 * ControlOverride.h
 *
 *  Created on: Nov 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CONTROLOVERRIDE_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CONTROLOVERRIDE_H_
#include "uavAP/Core/DataPresentation/APDataPresentation/SerializeCustom.h"

struct OverrideTarget : SerializeCustom
{
    double rollTarget;
    double pitchTarget;
    double velocityTarget;
    double climbRateTarget;
    double yawRateTarget;

    double rollOutput;
    double pitchOutput;
    double yawOutput;
    double throttleOutput;
    double flapOutput;
    OverrideTarget(): rollTarget(0), pitchTarget(0), velocityTarget(0), climbRateTarget(0), yawRateTarget(0), rollOutput(0), pitchOutput(0), yawOutput(0), throttleOutput(0), flapOutput(0) {}
};

struct OverrideActivation : SerializeCustom
{
    bool activate;

    bool overrideRollTarget;
    bool overridePitchTarget;
    bool overrideVelocityTarget;
    bool overrideClimbRateTarget;
    bool overrideYawRateTarget;

    bool overrideRollOutput;
    bool overridePitchOutput;
    bool overrideYawOutput;
    bool overrideThrottleOutput;
    bool overrideFlapOutput;
    OverrideActivation(): activate(false), overrideRollTarget(false), overridePitchTarget(false),
        overrideVelocityTarget(false), overrideClimbRateTarget(false), overrideYawRateTarget(false), overrideRollOutput(false),
        overridePitchOutput(false), overrideYawOutput(false), overrideThrottleOutput(false), overrideFlapOutput(false) {}
};

struct ControlOverride : SerializeCustom
{
    bool overrideManeuverPlanner;
    OverrideTarget target;
    OverrideActivation activation;
    ControlOverride(): overrideManeuverPlanner(false) {}
};




#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CONTROLOVERRIDE_H_ */
