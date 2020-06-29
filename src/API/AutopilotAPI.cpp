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
	if (runner.runAllStages())
	{
		CPSLOG_ERROR << "Run stages failed";
	}
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
