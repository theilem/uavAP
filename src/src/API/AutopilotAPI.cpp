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
 * AutopilotAPI.cpp
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/InertialFrame.h>
#include <uavAP/Core/Runner/SimpleRunner.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include "uavAP/API/AutopilotAPI.hpp"
#include "uavAP/API/APIHelper.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include <functional>

AutopilotAPI::AutopilotAPI() :
		localFrame_(0)
{
}

boost::signals2::connection
AutopilotAPI::subscribeOnControllerOut(const OnControllerOut::slot_type& slot)
{
	return onControllerOut_.connect(slot);
}

boost::signals2::connection
AutopilotAPI::subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot)
{
	return onAdvancedControl_.connect(slot);
}

void
AutopilotAPI::setSensorData(const SensorData& sd)
{
	SensorData data = sd;
	changeFrame(InertialFrame(), localFrame_, data);
	sensorDataPublisher_.publish(data);
}

void
AutopilotAPI::initialize()
{
	auto ipc = aggregator_.getOne<IPC>();
	sensorDataPublisher_ = ipc->publish<SensorData>("sensor_data");
	tryConnectControllerOut();
	tryConnectAdvancedControl();
	tryConnectLocalFrame();
	SimpleRunner runner(aggregator_);
	runner.runAllStages();
}

void
AutopilotAPI::tryConnectControllerOut()
{
	APLOG_DEBUG << "Try connect to controller out.";
	auto ipc = aggregator_.getOne<IPC>();
	controllerOutSubscription_ = ipc->subscribe<ControllerOutput>("actuation",
			std::bind(&AutopilotAPI::onControllerOut, this, std::placeholders::_1));
	if (!controllerOutSubscription_.connected())
	{
		APLOG_DEBUG << "Was not able to subscribe to actuation. Try again in 1sec.";
		auto scheduler = aggregator_.getOne<IScheduler>();
		scheduler->schedule(std::bind(&AutopilotAPI::tryConnectControllerOut, this), Seconds(1));
	}
}

void
AutopilotAPI::tryConnectAdvancedControl()
{
	auto ipc = aggregator_.getOne<IPC>();
	advancedControlSubscription_ = ipc->subscribe<AdvancedControl>("advanced_control_maneuver",
			std::bind(&AutopilotAPI::onAdvancedControl, this, std::placeholders::_1));
	if (!advancedControlSubscription_.connected())
	{
		auto scheduler = aggregator_.getOne<IScheduler>();
		scheduler->schedule(std::bind(&AutopilotAPI::tryConnectAdvancedControl, this), Seconds(1));
	}
}

void
AutopilotAPI::tryConnectLocalFrame()
{
	auto ipc = aggregator_.getOne<IPC>();
	localFrameSubscription_ = ipc->subscribe<VehicleOneFrame>("local_frame",
			std::bind(&AutopilotAPI::onLocalFrame, this, std::placeholders::_1));
	if (!localFrameSubscription_.connected())
	{
		auto scheduler = aggregator_.getOne<IScheduler>();
		scheduler->schedule(std::bind(&AutopilotAPI::tryConnectLocalFrame, this), Seconds(1));
	}
}

void
AutopilotAPI::onControllerOut(const ControllerOutput& control)
{
	onControllerOut_(control);
}

void
AutopilotAPI::onAdvancedControl(const AdvancedControl& control)
{
	onAdvancedControl_(control);
}

void
AutopilotAPI::onLocalFrame(const VehicleOneFrame& frame)
{
	localFrame_ = frame;
}

void
AutopilotAPI::configure(const Configuration& config)
{
	APIHelper helper;
	aggregator_ = helper.createAggregation(config);

}
