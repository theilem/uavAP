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

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_

#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

class SensorData;
class ControllerTarget;
class ControllerOutput;
class OverrideTarget;

class ManeuverCascade: public IPIDCascade
{
public:

	ManeuverCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
			ControllerTarget* target, ControllerOutput* output);

	bool
	configure(const Configuration& config);

	bool
	tunePID(PIDs pid, const Control::PIDParameters& params) override;

	bool
	tuneRollBounds(double min, double max) override;

	bool
	tunePitchBounds(double min, double max) override;

	std::map<PIDs, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	void
	setManeuverOverride(const Override& override);

private:

	SensorData* sensorData_;
	ControllerTarget* controllerTarget_;
	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> outputs_;

	std::shared_ptr<Control::Constraint<>> rollConstraint_;
	std::shared_ptr<Control::Constraint<>> pitchConstraint_;

	double hardRollConstraint_;
	double hardPitchConstraint_;
	double beta_;
	double rollTarget_;
};
#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_DETAIL_MANEUVERCASCADE_H_UVERCASCADE_H_ */
