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
 * RatePIDController.h
 *
 *  Created on: Sep 15, 2017
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_RATEPIDCONTROLLER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_RATEPIDCONTROLLER_H_

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"

class RateCascade;

class IScheduler;

class IPC;

class ISensingActuationIO;

class DataHandling;

class Packet;

class DataPresentation;


class RatePIDController
		: public IPIDController,
		  public AggregatableObject<ISensingActuationIO, IScheduler, DataHandling, IPC, DataPresentation>,
		  public IRunnableObject
{
public:

	static constexpr TypeId typeId = "rate";

	RatePIDController();

	static std::shared_ptr<RatePIDController>
	create(const Configuration& config);

	bool
	configure(const Configuration& config) override;

	bool
	run(RunStage stage) override;

	void
	setControllerTarget(const ControllerTarget& target) override;

	ControllerOutput
	getControllerOutput() override;

	std::shared_ptr<IPIDCascade>
	getCascade();

private:

	void
	calculateControl();

	void
	onOverridePacket(const Packet& packet);

	void
	tunePID(const PIDTuning& params);


	Mutex controllerTargetMutex_;
	ControllerTarget controllerTarget_;
	SensorData sensorData_;
	Vector3 velocityInertial_;
	Vector3 accelerationInertial_;
	ControllerOutput controllerOutput_;

	Publisher<ControllerOutput> controllerOutputPublisher_;
	Publisher<Packet> pidStatiPublisher_;
	Subscription overrideSubscription_;

	std::shared_ptr<RateCascade> pidCascade_;

};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_RATEPIDCONTROLLER_RATEPIDCONTROLLER_H_ */
