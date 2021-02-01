//
// Created by seedship on 1/30/21.
//

#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController.h"

PitchStateSpaceController::PitchStateSpaceController()
{
}

void
PitchStateSpaceController::setControllerTarget(const ControllerTarget& target)
{

}

ControllerOutput
PitchStateSpaceController::getControllerOutput()
{
	return ControllerOutput();
}

bool
PitchStateSpaceController::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			break;
		}
		case RunStage::NORMAL:
		{
			cascade_ = std::make_unique<PitchStateSpaceCascade>(params.a.value, params.b.value, params.k.value);
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
	return ConfigurableObject::configure(config);
}
