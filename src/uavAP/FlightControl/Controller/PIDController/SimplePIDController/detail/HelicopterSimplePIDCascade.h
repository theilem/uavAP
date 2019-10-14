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
 * HelicopterSimplePIDCascade.h
 *
 *  Created on: Sep 1, 2017
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_DETAIL_HELICOPTERSIMPLEPIDCASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_DETAIL_HELICOPTERSIMPLEPIDCASCADE_H_

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"

class HelicopterSimplePIDCascade: public IPIDCascade
{
public:

	HelicopterSimplePIDCascade(SensorData* sensorData, ControllerTarget* target,
			ControllerOutput* output);

	bool
	configure(const Configuration& config);

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

private:

	Control::ControlEnvironment controlEnv_;

	std::shared_ptr<Control::PID> velocityXPID_;
	std::shared_ptr<Control::PID> velocityYPID_;
	std::shared_ptr<Control::PID> climbRatePID_;
	std::shared_ptr<Control::PID> rollPID_;
	std::shared_ptr<Control::PID> pitchPID_;
	std::shared_ptr<Control::PID> yawRatePID_;

	std::shared_ptr<Control::Constraint> rollConstraint_;
	std::shared_ptr<Control::Constraint> pitchConstraint_;

	double hardRollConstraint_;
	double hardPitchConstraint_;
	double climbRateOffset_;
	double throttleTarget_;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_DETAIL_HELICOPTERSIMPLEPIDCASCADE_H_ */
