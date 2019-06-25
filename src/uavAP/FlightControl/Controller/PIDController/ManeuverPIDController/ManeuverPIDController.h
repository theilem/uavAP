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
 * ManeuverPIDController.h
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_MANEUVERPIDCONTROLLER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_MANEUVERPIDCONTROLLER_H_

#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/Object/AggregatableObject.hpp>
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include <mutex>

class ManeuverCascade;
class ISensingActuationIO;
class IScheduler;
class IPC;
class Packet;

class ManeuverPIDController: public IPIDController, public AggregatableObject<IPC, IScheduler, ISensingActuationIO>, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "maneuver";

	ManeuverPIDController();

	static std::shared_ptr<ManeuverPIDController>
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
	getCascade() override;

private:

	void
	calculateControl();

	void
	onOverridePacket(const Packet& packet);

	Mutex controllerTargetMutex_;
	ControllerTarget controllerTarget_;
	SensorData sensorData_;
	Vector3 velocityInertial_;
	Vector3 accelerationInertial_;
	ControllerOutput controllerOutput_;

	Publisher controllerOutputPublisher_;
	Subscription overrideSubscription_;

	std::shared_ptr<ManeuverCascade> pidCascade_;

};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERPIDCONTROLLER_MANEUVERPIDCONTROLLER_H_ */
