//
// Created by seedship on 1/30/21.
//

#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

PitchStateSpaceController::PitchStateSpaceController() : cascade_(sensorDataENU_, sensorDataNED_, target_, output_)
{
}

void
PitchStateSpaceController::setControllerTarget(const ControllerTarget& target)
{

	auto io = get<ISensingActuationIO>();
	if (!io)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}
	target_ = target;

	sensorDataENU_ = io->getSensorData();
	sensorDataNED_ = sensorDataENU_;
	NED::convert(sensorDataNED_);
	directionalConversion(sensorDataNED_.velocity, sensorDataNED_.attitude, Frame::BODY, Orientation::NED);

	io->setControllerOutput(getControllerOutput());
}

ControllerOutput
PitchStateSpaceController::getControllerOutput()
{
	Lock l(cascadeMutex_);
	cascade_.evaluate();
	return output_;
}

bool
PitchStateSpaceController::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<ISensingActuationIO, IScheduler, IPC, DataPresentation>())
			{
				CPSLOG_ERROR << "Missing Dependencies";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "DataHandling not set. Debugging disabled.";
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();
			overrideSubscription_ = ipc->subscribeOnPackets("override",
															std::bind(&PitchStateSpaceController::onOverridePacket,
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
											   std::bind(&PitchStateSpaceController::tunePID, this,
														 std::placeholders::_1));
				dh->addTriggeredStatusFunction<PIDParams, DataRequest>(
						std::bind(&PitchStateSpaceCascade::getPIDParams, &cascade_, std::placeholders::_1),
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

bool
PitchStateSpaceController::configure(const Configuration& config)
{
	cascade_.configure(config);
//	ParameterRef<PitchStateSpaceCascade> cascade(cascade_, "cascade", true);

	return ConfigurableObject::configure(config);
}

void
PitchStateSpaceController::onOverridePacket(const Packet& packet)
{
	auto dp = get<DataPresentation>();
	auto override = dp->deserialize<Override>(packet);
	Lock l(cascadeMutex_);
	cascade_.setManeuverOverride(override);
}

void
PitchStateSpaceController::tunePID(const PIDTuning& tune)
{
	Lock l(cascadeMutex_);
	cascade_.tunePID(static_cast<PIDs>(tune.pid), tune.params);
}