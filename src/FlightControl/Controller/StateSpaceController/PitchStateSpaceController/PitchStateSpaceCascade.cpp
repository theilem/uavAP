//
// Created by seedship on 1/30/21.
//

#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"

PitchStateSpaceCascade::PitchStateSpaceCascade(const std::vector<FloatingType>& a, const std::vector<FloatingType>& b,
											   const std::vector<FloatingType>& k) :
		a_(a.data()), b_(b.data()), k_(k.data())
{
	CPSLOG_WARN << a_;
	CPSLOG_WARN << b_;
	CPSLOG_WARN << k_;
}

//PitchStateSpaceCascade::PitchStateSpaceCascade(SensorData* sensorData, PitchStateSpaceControllerTarget* target,
//											   ControllerOutput* output, std::vector<FloatingType> k) :
//											   controlEnv_(&sensorData_->timestamp), K_matrix(k.data())
//{
//	CPSLOG_TRACE << "Create PitchStateSpaceCascade";
//
//	Control::PIDParameters defaultParams;
//
//	/* Roll Control */
//	auto rollTarget = controlEnv_.addInput(&rollTarget_);
//	rollConstraint_ = controlEnv_.addConstraint(rollTarget, -hardRollConstraint_ * M_PI / 180.0,
//												hardRollConstraint_ * M_PI / 180.0);
//
//	auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]);
//	auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[1]);
//
//	auto rollPID = controlEnv_.addPID(rollConstraint_, rollInput, rollRateInput, defaultParams);
//
//	/* Roll Output */
//
//	auto rollOutConstraint = controlEnv_.addConstraint(rollPID, -1, 1);
//	auto rollOut = controlEnv_.addOutput(rollOutConstraint, &output->rollOutput);
//
//	/* Rudder Output */
//	auto rudderBeta = controlEnv_.addInput(&beta_);
//	auto rudderTarget = controlEnv_.addConstant(0);
//
//	auto rudderPID = controlEnv_.addPID(rudderTarget, rudderBeta, defaultParams);
//	auto invertedRudder = controlEnv_.addGain(rudderPID, -1);
//	auto constraintYawOut = controlEnv_.addConstraint(invertedRudder, -1, 1);
//
//	auto yawOut = controlEnv_.addOutput(constraintYawOut, &output->yawOutput);
//
//
//	pids_.insert(std::make_pair(PIDs::RUDDER, rudderPID));
//	pids_.insert(std::make_pair(PIDs::ROLL, rollPID));
//
//	outputs_.insert(std::make_pair(ControllerOutputs::YAW, yawOut));
//	outputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOut));
//}