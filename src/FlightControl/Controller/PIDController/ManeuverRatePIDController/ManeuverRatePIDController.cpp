/*
 * ManeuverRatePIDController.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDController.h>

#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRateCascade.h>
#include <uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h>
#include <uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

ManeuverRatePIDController::ManeuverRatePIDController() :
		cascade_(sensorData_, target_, output_)
{
}

bool
ManeuverRatePIDController::configure(const Configuration& config)
{

	PropertyMapper<Configuration> pm(config);

	configureParams(pm);

	return pm.map();
}

bool
ManeuverRatePIDController::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<ISensingActuationIO, IScheduler, IPC, DataPresentation>())
			{
				CPSLOG_ERROR << "ManeuverRatePIDController: Missing Dependencies";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "ManeuverPIDController: DataHandling not set. Debugging disabled.";
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();
			overrideSubscription_ = ipc->subscribeOnPackets("override",
															std::bind(&ManeuverRatePIDController::onOverridePacket,
																	  this,
																	  std::placeholders::_1));

			if (!overrideSubscription_.connected())
			{
				CPSLOG_DEBUG << "Override not present.";
			}

			if (auto dh = get<DataHandling>())
			{
				dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
						std::bind(&IPIDCascade::getPIDStatus, &cascade_), Content::PID_STATUS);
				dh->subscribeOnData<PIDTuning>(Content::TUNE_PID,
												  std::bind(&ManeuverRatePIDController::tunePID, this,
															std::placeholders::_1));
				dh->addTriggeredStatusFunction<PIDParams, DataRequest>(
						std::bind(&ManeuverRateCascade::getPIDParams, &cascade_, std::placeholders::_1),
						Content::PID_PARAMS, Content::REQUEST_DATA);

			}

			break;
		}
		case RunStage::FINAL:
		{
			break;
		}
		default:
		{
			break;
		}
	}

	return false;
}

void
ManeuverRatePIDController::setControllerTarget(const ControllerTarget& target)
{

	auto io = get<ISensingActuationIO>();
	if (!io)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}
	target_ = target;

	sensorData_ = io->getSensorData();

	io->setControllerOutput(getControllerOutput());

}

ControllerOutput
ManeuverRatePIDController::getControllerOutput()
{
	Lock l(cascadeMutex_);
	cascade_.evaluate();
	return output_;
}

void
ManeuverRatePIDController::onOverridePacket(const Packet& packet)
{
	auto dp = get<DataPresentation>();
	auto override = dp->deserialize<Override>(packet);
	Lock l(cascadeMutex_);
	cascade_.setManeuverOverride(override);
}

void
ManeuverRatePIDController::tunePID(const PIDTuning& tune)
{
	Lock l(cascadeMutex_);
	cascade_.tunePID(static_cast<PIDs>(tune.pid), tune.params);
}

void
ManeuverRatePIDController::setThrottleLimit(FloatingType maxThrottle)
{
	Lock l(cascadeMutex_);
	cascade_.setThrottleLimit(maxThrottle);
}
