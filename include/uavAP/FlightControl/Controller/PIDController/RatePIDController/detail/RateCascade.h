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

#include "uavAP/FlightControl/Controller/ControllerConstraint.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

struct SensorData;
struct ControllerTarget;
struct ControllerOutput;
struct OverrideTarget;

class RateCascade: public IPIDCascade
{
public:

	RateCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
			ControllerTarget* target, ControllerOutput* output, ServoData* servoData = nullptr);

	bool
	configure(const Configuration& config);

	bool
	tunePID(PIDs pid, const Control::PIDParameters& params) override;

	bool
	tuneRollBounds(FloatingType min, FloatingType max) override;

	bool
	tunePitchBounds(FloatingType min, FloatingType max) override;

	bool
	tuneRollRateBounds(FloatingType min, FloatingType max);

	bool
	tunePitchRateBounds(FloatingType min, FloatingType max);

	PIDStati
	getPIDStatus() const override;

	void
	evaluate() override;

private:

	SensorData* sensorData_;
	ControllerTarget* controllerTarget_;
	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> outputs_;
	std::map<ControllerOutputsWaveforms, std::shared_ptr<Control::Output>> outputWaveforms_;
	std::map<ControllerConstraints, std::shared_ptr<Control::Constraint<>>> constraints_;

	std::shared_ptr<Control::ManualSwitch> throttleManualSwitch_;

	std::shared_ptr<Control::Constraint<>> rollTargetConstraint_;
	std::shared_ptr<Control::Constraint<>> rollRateTargetConstraint_;
	std::shared_ptr<Control::Constraint<>> rollOutputConstraint_;
	std::shared_ptr<Control::Constraint<>> pitchTargetConstraint_;
	std::shared_ptr<Control::Constraint<>> pitchRateTargetConstraint_;
	std::shared_ptr<Control::Constraint<>> pitchOutputConstraint_;
	std::shared_ptr<Control::Constraint<>> yawOutputConstraint_;
	std::shared_ptr<Control::Constraint<>> throttleOutputConstraint_;

	FloatingType hardRollConstraint_;
	FloatingType hardRollRateConstraint_;
	FloatingType hardPitchConstraint_;
	FloatingType hardPitchRateConstraint_;
	FloatingType rollConstraint_;
	FloatingType rollRateConstraint_;
	FloatingType rollOutConstraint_;
	FloatingType pitchConstraint_;
	FloatingType pitchRateConstraint_;
	FloatingType pitchOutConstraint_;
	FloatingType yawOutConstraint_;
	FloatingType throttleOutConstraint_;
	FloatingType beta_;
	FloatingType rollTarget_;

	bool useRPMController_;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_DETAIL_RATECASCADE_H_ */
