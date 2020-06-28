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
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include "uavAP/API/AutopilotAPI.hpp"
#include "uavAP/API/APIHelper.h"
#include <functional>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Synchronization/SimpleRunner.h>

void
AutopilotAPI::configure(const Configuration& config)
{
	aggregator_ = APIHelper::createAggregation(config);

}

void
AutopilotAPI::initialize()
{
	SimpleRunner runner(aggregator_);
	runner.runAllStages();
}

boost::signals2::connection
AutopilotAPI::subscribeOnControllerOut(const OnControllerOut::slot_type& slot)
{
	return aggregator_.getOne<AggregatableAutopilotAPI>()->subscribeOnControllerOut(slot);
}

boost::signals2::connection
AutopilotAPI::subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot)
{
	return aggregator_.getOne<AggregatableAutopilotAPI>()->subscribeOnAdvancedControl(slot);
}

void
AutopilotAPI::setSensorData(const SensorData& sd)
{
	return aggregator_.getOne<AggregatableAutopilotAPI>()->setSensorData(sd);
}

void
AutopilotAPI::setServoData(const ServoData& sd)
{
	return aggregator_.getOne<AggregatableAutopilotAPI>()->setServoData(sd);
}

void
AutopilotAPI::setPowerData(const PowerData& pd)
{
	return aggregator_.getOne<AggregatableAutopilotAPI>()->setPowerData(pd);
}
