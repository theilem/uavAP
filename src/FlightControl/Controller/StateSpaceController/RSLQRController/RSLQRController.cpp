//
// Created by seedship on 7/23/21.
//

#include <uavAP/FlightControl/FlightControlHelper.h>
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/RSLQRController/RSLQRController.h"

#include "uavAP/Core/DataHandling/DataHandling.h"

RSLQRController::RSLQRController(): controlEnv_(&sd_ned_.timestamp)
{
}

bool
RSLQRController::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			populateControlEnv();
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
			{
				for (const auto& it:ctrlEnvOutputs_)
				{
					oh->registerOverride("output/" + EnumMap<ControllerOutputs>::convert(it.first),
													  it.second->getOutputOverridableValue());
					oh->registerOverride("save_trim/" + EnumMap<ControllerOutputs>::convert(it.first),
													  it.second->getSaveTrimOverridableValue());
					oh->registerOverride("apply_trim/" + EnumMap<ControllerOutputs>::convert(it.first),
													  it.second->getApplyTrimOverridableValue());
				}
			}
			else
				CPSLOG_DEBUG << "OverrideHandler not set. Overrides disabled.\n";
			break;
		}
		case RunStage::NORMAL:
		{

			if (auto dh = get<DataHandling>())
			{
//				dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
//						[this](){return getStatus();}, Content::PID_STATUS);
				dh->addTriggeredStatusFunction<PIDParams, DataRequest>([this](const DataRequest& request){return getDummyPIDParams(request);},
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
RSLQRController::setControllerTarget(const ControllerTarget& target)
{
	auto io = get<ISensingIO>();
	auto actIo = get<IActuationIO>();
	if (!io || !actIo)
	{
		CPSLOG_ERROR << "Cannot calculate control, IO missing";
		return;
	}

	Lock sdl(lock_);
	sd_ned_ = io->getSensorData();
	NED::convert(sd_ned_, Frame::BODY);
	ct_ = target;
	sdl.unlock();

	auto state = getState();
	stateOut_ = -params.k() * state;

	controlEnv_.evaluate();

	actIo->setControllerOutput(co_);
}

VectorN<12>
RSLQRController::getState()
{
	if(sd_ned_.orientation != Orientation::NED) {
		NED::convert(sd_ned_, Frame::BODY);
	}

	VectorN<12> state;

	Lock sdl(lock_);
	FramedVector3 inertialAngularRate = sd_ned_.angularRate;
	angularConversion(inertialAngularRate, sd_ned_.attitude, Frame::INERTIAL, sd_ned_.orientation);

	// e_u
	state[0] = sd_ned_.velocity.x() - ct_.velocity;
	// e_theta
	state[1] = sd_ned_.attitude.y() - ct_.climbAngle;
	// e_v
	state[2] = sd_ned_.velocity.y();
	// e_psi
	state[3] = sd_ned_.attitude.x() - ct_.yawRate;
	// u_dot
	state[4] = sd_ned_.acceleration.x();
	// w_dot
	state[5] = sd_ned_.acceleration.z();
	// q_dot
	state[6] = sd_ned_.angularRate.y();
	// theta_dot
	state[7] = inertialAngularRate.y();
	// v_dot
	state[8] = sd_ned_.acceleration.y();
	// p_dot
	state[9] = sd_ned_.angularRate.x();
	// r_dot
	state[10] = sd_ned_.angularRate.z();
	// phi_dot
	state[11] = inertialAngularRate.x();
	sdl.unlock();

	return state;
}

void
RSLQRController::populateControlEnv()
{
	auto dElevator = controlEnv_.addInput(&stateOut_[0]);
	auto dThrottle = controlEnv_.addInput(&stateOut_[1]);
	auto dAileron = controlEnv_.addInput(&stateOut_[2]);
	auto dRudder = controlEnv_.addInput(&stateOut_[3]);

	auto elevInt = controlEnv_.addIntegrator(dElevator, params.elevatorParams());
	auto throtInt = controlEnv_.addIntegrator(dThrottle, params.throttleParams());
	auto ailInt = controlEnv_.addIntegrator(dAileron, params.aileronParams());
	auto rudInt = controlEnv_.addIntegrator(dRudder, params.rudderParams());

	auto elevatorOut = controlEnv_.addOutput(elevInt, &co_.pitchOutput);
	auto throttleOut = controlEnv_.addOutput(throtInt, &co_.throttleOutput);
	auto aileronOut = controlEnv_.addOutput(ailInt, &co_.rollOutput);
	auto rudderOut = controlEnv_.addOutput(rudInt, &co_.yawOutput);

	ctrlEnvOutputs_.emplace(ControllerOutputs::PITCH, elevatorOut);
	ctrlEnvOutputs_.emplace(ControllerOutputs::THROTTLE, throttleOut);
	ctrlEnvOutputs_.emplace(ControllerOutputs::ROLL, aileronOut);
	ctrlEnvOutputs_.emplace(ControllerOutputs::YAW, rudderOut);
}

std::map<PIDs, PIDStatus>
RSLQRController::getStatus() const
{
	std::map<PIDs, PIDStatus> ans;
	ans[PIDs::PITCH] = PIDStatus(radToDeg(ct_.climbAngle), radToDeg(sd_ned_.attitude.y()));
	ans[PIDs::VELOCITY_X] = PIDStatus(ct_.velocity, sd_ned_.velocity.x());
	ans[PIDs::ROLL] = PIDStatus(radToDeg(ct_.yawRate), radToDeg(sd_ned_.attitude.x()));
	ans[PIDs::VELOCITY_Y] = PIDStatus(0, sd_ned_.velocity.y());
	return ans;
}

Optional<PIDParams>
RSLQRController::getDummyPIDParams(const DataRequest& request)
{
	if (request != DataRequest::PID_PARAMS)
		return std::nullopt;

	PIDParams ans;
	// Controller targets
	ans[PIDs::PITCH];
	ans[PIDs::VELOCITY_X];
	ans[PIDs::ROLL];
	ans[PIDs::VELOCITY_Y];
	// Planner Targets
	ans[PIDs::ALTITUDE];
	ans[PIDs::HEADING];
	return ans;
}
