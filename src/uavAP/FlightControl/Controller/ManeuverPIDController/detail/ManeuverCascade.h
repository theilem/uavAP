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
 * ManeuverCascade.h
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/MissionControl/MissionPlanner/ControlOverride.h"

class SensorData;
class ControllerTarget;
class ControllerOutput;
class OverrideTarget;

class ManeuverCascade: public IPIDCascade
{
public:

    ManeuverCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
                    ControllerTarget* target, ControllerOutput* output, OverrideTarget* manTarget);

    bool
    configure(const boost::property_tree::ptree& config) override;

    bool
    tunePID(int pid, const Control::PID::Parameters& params) override;

    bool
    tuneRollBounds(double min, double max) override;

    bool
    tunePitchBounds(double min, double max) override;

    std::map<int, PIDStatus>
    getPIDStatus() override;

    void
    evaluate() override;

    void
    setManeuverOverride(const OverrideActivation& state);

private:

    SensorData* sensorData_;
    Control::ControlEnvironment controlEnv_;

    std::shared_ptr<Control::PID> velocityPID_;
    std::shared_ptr<Control::PID> climbAnglePID_;
    std::shared_ptr<Control::PID> pitchPID_;
    std::shared_ptr<Control::PID> rollPID_;
    std::shared_ptr<Control::PID> yawRatePID_;
    std::shared_ptr<Control::PID> rudderPID_;

    //Target override switches
    std::shared_ptr<Control::ManualSwitch> switchRollTarget_;
    std::shared_ptr<Control::ManualSwitch> switchPitchTarget_;
    std::shared_ptr<Control::ManualSwitch> switchVelocityTarget_;
    std::shared_ptr<Control::ManualSwitch> switchClimbRateTarget_;
    std::shared_ptr<Control::ManualSwitch> switchYawRateTarget_;

    //Output override switches
    std::shared_ptr<Control::ManualSwitch> switchRollOutput_;
    std::shared_ptr<Control::ManualSwitch> switchPitchOutput_;
    std::shared_ptr<Control::ManualSwitch> switchYawOutput_;
    std::shared_ptr<Control::ManualSwitch> switchThrottleOutput_;
    std::shared_ptr<Control::ManualSwitch> switchFlapOutput_;

    std::shared_ptr<Control::Constraint> rollConstraint_;
    std::shared_ptr<Control::Constraint> pitchConstraint_;

    double hardRollConstraint_;
    double hardPitchConstraint_;
    double beta_;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_ */
