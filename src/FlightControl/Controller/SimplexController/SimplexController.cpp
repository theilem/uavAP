//
// Created by seedship on 4/30/21.
//

#include <uavAP/Core/Orientation/NED.h>
#include "uavAP/FlightControl/Controller/SimplexController/SimplexController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"
#include <uavAP/Core/DataHandling/DataHandling.h>

SimplexController::SimplexController() : cascade_(sensorDataENU_, sensorDataNED_, target_, output_)
{

}

void
SimplexController::setControllerTarget(const ControllerTarget& target)
{
	auto io = get<ISensingIO>();
	auto actIo = get<IActuationIO>();
	if (!io || !actIo)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}
	lastTarget_ = target_;
	target_ = target;

	sensorDataENU_ = io->getSensorData();
	sensorDataNED_ = sensorDataENU_;
	NED::convert(sensorDataNED_, Frame::BODY);

	auto xt = _generateState();
	// TODO calculate simplex here
	auto dtau = params.a.value * xt + params.b.value * _generateU(lastTarget_);
	auto xt_tau = xt + params.controllerPeriodMS.value / 1E3 * dtau;

	double simplexTerm = xt_tau.transpose() * params.p.value * xt_tau;

	if (simplexTerm > 1)
	{ // Safety Controller
		target_.velocity = params.rU.value;
		target_.climbAngle = std::clamp<FloatingType>(params.kPitch.value.dot(xt), params.maxPitchTarget.value, params.minPitchTarget.value);
		std::cout << "Using Safety Controller\n";
	}
	std::cout << "State[0]: (u) " << xt[0] << "\n";
	std::cout << "State[1]: (w) " << xt[1] << "\n";
	std::cout << "State[2]: (q) " << radToDeg(xt[2]) << "\n";
	std::cout << "State[3]: (θ) " << radToDeg(xt[3]) << "\n";
	std::cout << "State[4]: (Ep) " << xt[4] << "\n";
	std::cout << "State[5]: (Ev) " << xt[5] << "\n";
	std::cout << "State[6]: (dh) " << xt[6] << "\n";
	std::cout << "Pitch Target (degrees): " << radToDeg(target_.climbAngle) << "\n";
	std::cout << "U Target: " << target_.velocity << "\n";
	std::cout << "θ Target: " << radToDeg(target_.climbAngle) << "\n";
	std::printf("Simplex Term: %lf\n", simplexTerm);


	actIo->setControllerOutput(getControllerOutput());
}

bool
SimplexController::configure(const Configuration& config)
{
	cascade_.configure(config);
	return ConfigurableObject::configure(config);
}

ControllerOutput
SimplexController::getControllerOutput()
{
	Lock l(cascadeMutex_);
	cascade_.evaluate();
	return output_;
}

bool
SimplexController::run(RunStage stage)
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
				dh->addStatusFunction<std::map<PIDs, PIDStatus>>([this]()
																 { return cascade_.getPIDStatus(); },
																 Content::PID_STATUS);
				dh->subscribeOnData<PIDTuning>(Content::TUNE_PID,
											   std::bind(&SimplexController::tunePID, this,
														 std::placeholders::_1));
				dh->addTriggeredStatusFunction<PIDParams, DataRequest>([this](const DataRequest& request)
																	   { return cascade_.getPIDParams(request); },
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

VectorN<7>
SimplexController::generateState() const
{
	Lock l(cascadeMutex_);
	return _generateState();
}

void
SimplexController::tunePID(const PIDTuning& tune)
{
	Lock l(cascadeMutex_);
	cascade_.tunePID(static_cast<PIDs>(tune.pid), tune.params);
}

VectorN<7>
SimplexController::_generateState() const
{
	Lock l(cascadeMutex_);
	//TODO use initializer list when Eigen 3.4 is released
	VectorN<7> state;// = {controller->getState(), positionDeviation.z()};
	auto val = cascade_.getState();
	state[0] = val[0];
	state[1] = val[1];
	state[2] = val[2];
	state[3] = val[3];
	state[4] = val[4];
	state[5] = val[5];
	state[6] = sensorDataENU_.position.z() - params.safetyAlt.value;
	return state;
}

Vector2
SimplexController::_generateU(const ControllerTarget& t) const
{
	return {t.climbAngle - params.rP.value, t.velocity - params.rU.value};
}
