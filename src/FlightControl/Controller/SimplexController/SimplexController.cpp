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

	auto xt = _generateState();
	auto dtau = params.a() * xt + params.b() * _generateU(target_);
	auto xt_tau = xt + params.controllerPeriodMS() / 1E3 * dtau;
	double simplexTerm = xt_tau.transpose() * params.p() * xt_tau;

	if (safetyController_)
	{ // Safety Controller
		executeSafetyControl(xt);
		if (std::chrono::system_clock::now() > switchTime_)
		{
			safetyController_ = false;
		}
	} else if (simplexTerm > 1) {
		executeSafetyControl(xt);
		switchTime_ = std::chrono::system_clock::now() + std::chrono::seconds(5);
		safetyController_ = true;
	}
	std::cout << "Safety Active: " << safetyController_ << "\n";
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
	std::cout << "Next Simplex Term: " << simplexTerm <<"\n";
	std::cout << "Current Simplex Term: " << xt.transpose() * params.p() * xt <<"\n";


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
	state[6] = params.stateSpaceAltitudeParams().safetyAltitude() - sensorDataENU_.position.z();
	return state;
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
