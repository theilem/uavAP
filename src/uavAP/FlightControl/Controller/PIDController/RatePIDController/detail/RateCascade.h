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
 * RateCascade.h
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_DETAIL_RATECASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_DETAIL_RATECASCADE_H_

#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

class SensorData;
class ControllerTarget;
class ControllerOutput;
class OverrideTarget;

class RateCascade: public IPIDCascade
{
public:

	RateCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
			ControllerTarget* target, ControllerOutput* output);

	bool
	configure(const boost::property_tree::ptree& config) override;

	bool
	tunePID(PIDs pid, const Control::PID::Parameters& params) override;

	bool
	tuneRollBounds(double min, double max) override;

	bool
	tunePitchBounds(double min, double max) override;

	bool
	tuneRollRateBounds(double min, double max);

	bool
	tunePitchRateBounds(double min, double max);

	PIDStati
	getPIDStatus() override;

	void
	evaluate() override;

	void
	setManeuverOverride(const Override& state);

private:

	SensorData* sensorData_;
	ControllerTarget* controllerTarget_;
	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> outputs_;

	std::shared_ptr<Control::Constraint> rollTargetConstraint_;
	std::shared_ptr<Control::Constraint> rollRateTargetConstraint_;
	std::shared_ptr<Control::Constraint> pitchTargetConstraint_;
	std::shared_ptr<Control::Constraint> pitchRateTargetConstraint_;

	double hardRollConstraint_;
	double hardRollRateConstraint_;
	double hardPitchConstraint_;
	double hardPitchRateConstraint_;
	double rollConstraint_;
	double rollRateConstraint_;
	double pitchConstraint_;
	double pitchRateConstraint_;
	double beta_;
	double rollTarget_;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_DETAIL_RATECASCADE_H_ */
