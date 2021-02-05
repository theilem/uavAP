//
// Created by seedship on 1/30/21.
//

#include "uavAP/Core/Orientation/ConversionUtils.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController.h"
#include "uavAP/Core/Orientation/NED.h"

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
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "ManeuverRatePIDController: Missing Dependencies";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			cascade_.printGains();
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
	return ConfigurableObject::configure(config);
}
