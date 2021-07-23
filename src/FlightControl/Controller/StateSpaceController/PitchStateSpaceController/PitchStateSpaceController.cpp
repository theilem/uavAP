//
// Created by seedship on 1/30/21.
//

#include "uavAP/Core/Orientation/ConversionUtils.h"
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController/PitchStateSpaceController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

PitchStateSpaceController::PitchStateSpaceController() : cascade_(sensorDataENU_, sensorDataNED_, target_, output_)
{
}

void
PitchStateSpaceController::setControllerTarget(const ControllerTarget& target)
{
	auto io = get<ISensingIO>();
	auto actIo = get<IActuationIO>();
	if (!io || !actIo)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}
	target_ = target;

	sensorDataENU_ = io->getSensorData();
	sensorDataNED_ = sensorDataENU_;
	NED::convert(sensorDataNED_, Frame::BODY);

	actIo->setControllerOutput(getControllerOutput());
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
			if (!checkIsSet<IScheduler, ISensingIO, IActuationIO>())
			{
				CPSLOG_ERROR << "Missing Dependencies";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "DataHandling not set. Debugging disabled.";
			}
			if (auto oh = get<OverrideHandler>())
				cascade_.registerOverrides(oh);
			else
				CPSLOG_DEBUG << "OverrideHandler not set. Overrides disabled.\n";
			break;
		}
		case RunStage::NORMAL:
		{

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

				dh->subscribeOnData<bool>(Content::CLEAR_LQR_INTEGRATOR, [this](const auto&)
				{
					Lock l(cascadeMutex_);
					cascade_.clearIntegrators();
				});
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
PitchStateSpaceController::tunePID(const PIDTuning& tune)
{
	Lock l(cascadeMutex_);
	cascade_.tunePID(static_cast<PIDs>(tune.pid), tune.params);
}

VectorN<6>
PitchStateSpaceController::getState() const
{
	Lock l(cascadeMutex_);
	return cascade_.getState();
}

FloatingType
PitchStateSpaceController::getUTrim() const
{
	Lock l(cascadeMutex_);
	return cascade_.getUTrim();
}

FloatingType
PitchStateSpaceController::getThetaTrim() const
{
	Lock l(cascadeMutex_);
	return cascade_.getThetaTrim();
}