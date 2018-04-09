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
 *  @file         PIDController.h
 *  @author Simon Yu, Mirco Theile
 *  @date      02 June 2017
 *  @brief      UAV Autopilot PID Controller Header File
 *
 *  Description
 */

#ifndef FLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_H_
#define FLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_H_

#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include <vector>
#include <memory>
#include <mutex>

#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/ChannelMixing/IChannelMixing.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Time.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"

class FlightControlData;
class IScheduler;

class PIDController: public IController, public IAggregatableObject, public IRunnableObject
{
public:

	PIDController();

	static std::shared_ptr<PIDController>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	bool
	run(RunStage stage) override;

	void
	setControllerTarget(const ControllerTarget& target) override;

	ControllerOutput
	getControllerOutput();

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

	std::shared_ptr<IPIDCascade>
	getCascade();

private:

	void
	calculateControl();

	ObjectHandle<SensingActuationIO> sensAct_;
	ObjectHandle<IScheduler> scheduler_;


	std::mutex controllerTargetMutex_;
	ControllerTarget controllerTarget_;
	SensorData sensorData_;
	Vector3 velocityInertial_;
	Vector3 accelerationInertial_;
	ControllerOutput controllerOutput_;

	std::shared_ptr<IPIDCascade> pidCascade_;

	bool airplane_;
};

#endif /* FLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_PIDFLIGHTCONTROLLER_H_ */
