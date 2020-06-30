//
// Created by seedship on 6/28/20.
//

#include "uavAP/API/AggregatableAutopilotAPI.h"
#include "uavAP/Core/Frames/InertialFrame.h"

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

AggregatableAutopilotAPI::AggregatableAutopilotAPI() :
		localFrame_(0)
{
}

boost::signals2::connection
AggregatableAutopilotAPI::subscribeOnControllerOut(const OnControllerOut::slot_type& slot)
{
	return onControllerOut_.connect(slot);
}

boost::signals2::connection
AggregatableAutopilotAPI::subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot)
{
	return onAdvancedControl_.connect(slot);
}

void
AggregatableAutopilotAPI::setSensorData(const SensorData& sd)
{
	SensorData data = sd;
	changeFrame(InertialFrame(), localFrame_, data);
	sensorDataPublisher_.publish(data);
}

void
AggregatableAutopilotAPI::setServoData(const ServoData& sd)
{
	servoDataPublisher_.publish(sd);
}

void
AggregatableAutopilotAPI::setPowerData(const PowerData& pd)
{
	powerDataPublisher_.publish(pd);
}

bool
AggregatableAutopilotAPI::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<IScheduler, IPC>())
			{
				CPSLOG_ERROR << "Dependencies missing";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();
			IPCOptions ipcOpts;
			ipcOpts.retry = true;

			sensorDataPublisher_ = ipc->publish<SensorData>("sensor_data");
			servoDataPublisher_ = ipc->publish<ServoData>("servo_data");
			powerDataPublisher_ = ipc->publish<PowerData>("power_data");

			ipc->subscribe<ControllerOutput>("actuation", std::bind(&AggregatableAutopilotAPI::onControllerOut, this,
																	std::placeholders::_1), ipcOpts);
			ipc->subscribe<AdvancedControl>("advanced_control_maneuver",
											std::bind(&AggregatableAutopilotAPI::onAdvancedControl, this,
													  std::placeholders::_1), ipcOpts);
			ipc->subscribe<VehicleOneFrame>("local_frame", std::bind(&AggregatableAutopilotAPI::onLocalFrame, this,
																	 std::placeholders::_1), ipcOpts);

			break;
		}
		default:
			break;
	}

	return false;
}

void
AggregatableAutopilotAPI::tryConnectControllerOut()
{
	CPSLOG_DEBUG << "Try connect to controller out.";
	auto ipc = get<IPC>();
	controllerOutSubscription_ = ipc->subscribe<ControllerOutput>("actuation",
																  std::bind(&AggregatableAutopilotAPI::onControllerOut, this,
																			std::placeholders::_1));
	if (!controllerOutSubscription_.connected())
	{
		CPSLOG_DEBUG << "Was not able to subscribe to actuation. Try again in 1sec.";
		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&AggregatableAutopilotAPI::tryConnectControllerOut, this), Seconds(1));
	}
}

void
AggregatableAutopilotAPI::tryConnectAdvancedControl()
{
	auto ipc = get<IPC>();
	advancedControlSubscription_ = ipc->subscribe<AdvancedControl>("advanced_control_maneuver",
																   std::bind(
																		   &AggregatableAutopilotAPI::onAdvancedControl,
																		   this, std::placeholders::_1));
	if (!advancedControlSubscription_.connected())
	{
		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&AggregatableAutopilotAPI::tryConnectAdvancedControl, this), Seconds(1));
	}
}

void
AggregatableAutopilotAPI::tryConnectLocalFrame()
{
	auto ipc = get<IPC>();
	localFrameSubscription_ = ipc->subscribe<VehicleOneFrame>("local_frame",
															  std::bind(&AggregatableAutopilotAPI::onLocalFrame, this,
																		std::placeholders::_1));
	if (!localFrameSubscription_.connected())
	{
		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&AggregatableAutopilotAPI::tryConnectLocalFrame, this), Seconds(1));
	}
}

void
AggregatableAutopilotAPI::onControllerOut(const ControllerOutput& control)
{
	onControllerOut_(control);
}

void
AggregatableAutopilotAPI::onAdvancedControl(const AdvancedControl& control)
{
	onAdvancedControl_(control);
}

void
AggregatableAutopilotAPI::onLocalFrame(const VehicleOneFrame& frame)
{
	localFrame_ = frame;
}