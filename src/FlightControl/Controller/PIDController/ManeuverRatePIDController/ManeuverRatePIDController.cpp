/*
 * ManeuverRatePIDController.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDController.h>

#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRateCascade.h>
#include <uavAP/FlightControl/SensingActuationIO/ISensingIO.h>
#include <uavAP/FlightControl/SensingActuationIO/IActuationIO.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/OverrideHandler/OverrideHandler.h>
#include <uavAP/Core/DataHandling/Content.hpp>
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
			if (!checkIsSet<ISensingIO, IActuationIO, IScheduler>())
			{
				CPSLOG_ERROR << "ManeuverRatePIDController: Missing Dependencies";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "ManeuverPIDController: DataHandling not set. Debugging disabled.";
			}

			if (auto oh = get<OverrideHandler>())
				cascade_.registerOverrides(oh);
			else
				CPSLOG_DEBUG << "ManeuverPIDController: OverrideHandler not set. Overrides disabled.";

			break;
		}
		case RunStage::NORMAL:
		{
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

	auto io = get<ISensingIO>();
	auto actIo = get<IActuationIO>();
	if (!io || !actIo)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}
	target_ = target;

	sensorData_ = io->getSensorData();

	actIo->setControllerOutput(getControllerOutput());

}

ControllerOutput
ManeuverRatePIDController::getControllerOutput()
{
	Lock l(cascadeMutex_);
	cascade_.evaluate();
	return output_;
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
