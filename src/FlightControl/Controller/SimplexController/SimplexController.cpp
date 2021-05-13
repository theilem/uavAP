//
// Created by seedship on 4/30/21.
//

#include <uavAP/Core/Orientation/NED.h>
#include "uavAP/FlightControl/Controller/SimplexController/SimplexController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"
#include <uavAP/Core/DataHandling/DataHandling.h>

SimplexController::SimplexController() : cascade_(sensorDataENU_, sensorDataNED_, target_, output_), safetyController_(true)
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
	target_ = target;

	sensorDataENU_ = io->getSensorData();
	sensorDataNED_ = sensorDataENU_;
	NED::convert(sensorDataNED_, Frame::BODY);

	auto xt = generateState();
	auto currentSimplexTerm = xt.transpose() * params.p() * xt;
	auto dtau = params.a() * xt + params.b() * _generateU(target_);
	auto xt_tau = xt + dtau / params.controllerFrequency();
	auto nextSimplexTerm = xt_tau.transpose() * params.p() * xt_tau;

	if (nextSimplexTerm > 1)
	{
		if (params.recoveryMode() == SimplexControllerParams::SIMPLEX_RECOVERYMODE_TIME)
		{
			switchTime_ = std::chrono::system_clock::now() + std::chrono::seconds(params.recoveryTime());
		}
		// SIMPLEX_RECOVERYMODE_VALUE does not need to be handled
		else if (params.recoveryMode() != SimplexControllerParams::SIMPLEX_RECOVERYMODE_VALUE)
		{
			CPSLOG_WARN << "Unknown Simplex Recovery Method defined: " << params.recoveryMode() << ". Changing to " << SimplexControllerParams::SIMPLEX_RECOVERYMODE_VALUE;
			params.recoveryMode() = SimplexControllerParams::SIMPLEX_RECOVERYMODE_VALUE;
		}
		safetyController_ = true;
	}
	if (safetyController_)
	{ // Safety Controller
		executeSafetyControl(xt);
		if (params.recoveryMode() == SimplexControllerParams::SIMPLEX_RECOVERYMODE_TIME)
		{
			if (std::chrono::system_clock::now() > switchTime_)
			{
				safetyController_ = false;
			}
		}
		else // SIMPLEX_RECOVERYMODE_VALUE
		{
			if (currentSimplexTerm < params.recoveryValue())
			{
				safetyController_ = false;
			}
		}
	}
//	std::cout << "Safety Active: " << safetyController_;
//	if (safetyController_) {
//		std::cout << " (Remaining time: " << std::chrono::duration_cast<std::chrono::milliseconds>(switchTime_ - std::chrono::system_clock::now()).count() / 1000.0 << ")";
//	}
//	std::cout << "\n";
//	std::cout << "State[0]: (u) " << xt[0] << "\n";
//	std::cout << "State[1]: (w) " << xt[1] << "\n";
//	std::cout << "State[2]: (q) " << radToDeg(xt[2]) << "\n";
//	std::cout << "State[3]: (θ) " << radToDeg(xt[3]) << "\n";
//	std::cout << "State[4]: (Ep) " << xt[4] << "\n";
//	std::cout << "State[5]: (Ev) " << xt[5] << "\n";
//	std::cout << "State[6]: (dh) " << xt[6] << "\n";
//	std::cout << "Pitch Target (degrees): " << radToDeg(target_.climbAngle) << "\n";
//	std::cout << "U Target: " << target_.velocity << "\n";
//	std::cout << "θ Target: " << radToDeg(target_.climbAngle) << "\n";
//	std::cout << "Next Simplex Term: " << simplexTerm <<"\n";
//	std::cout << "Current Simplex Term: " << xt.transpose() * params.p() * xt <<"\n";

#define SEP << "," <<
	std::cout << safetyController_ SEP xt[0] SEP xt[1] SEP xt[2] SEP xt[3] SEP xt[4] SEP xt[5] SEP xt[6] SEP target_.climbAngle SEP target_.velocity SEP nextSimplexTerm SEP currentSimplexTerm << "\n";
#undef SEP


	actIo->setControllerOutput(getControllerOutput());
}

ControllerOutput
SimplexController::getControllerOutput()
{
	Lock l(cascadeMutex_);
	cascade_.evaluate();
	return output_;
}

bool
SimplexController::configure(const Configuration& config)
{
	return ConfigurableObject::configure(config);
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

			cascade_.setParams(params.controllerParams());

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
	//TODO use initializer list when Eigen 3.4 is released
	VectorN<7> state;// = {controller->getState(), positionDeviation.z()};
	Lock l(cascadeMutex_);
	auto val = cascade_.getState();
	l.unlock();
	state[0] = val[0];
	state[1] = val[1];
	state[2] = val[2];
	state[3] = val[3];
	state[4] = val[4];
	state[5] = val[5];
	state[6] = params.stateSpaceAltitudeParams().safetyAltitude() - sensorDataENU_.position.z();
	return state;
}

void
SimplexController::tunePID(const PIDTuning& tune)
{
	Lock l(cascadeMutex_);
	cascade_.tunePID(static_cast<PIDs>(tune.pid), tune.params);
}

Vector2
SimplexController::_generateU(const ControllerTarget& t) const
{
	return {t.climbAngle - params.controllerParams().rP(), t.velocity - params.controllerParams().rU()};
}

void
SimplexController::executeSafetyControl(const VectorN<7>& state)
{
	target_.velocity = params.controllerParams().rU();
	target_.climbAngle = std::clamp<FloatingType>(params.stateSpaceAltitudeParams().k().dot(state) + params.controllerParams().rP(),
											   params.stateSpaceAltitudeParams().minPitchTarget(),
											   params.stateSpaceAltitudeParams().maxPitchTarget());
}
