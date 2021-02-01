//
// Created by seedship on 1/30/21.
//

#ifndef UAVAP_PITCHSTATESPACECASCADE_H
#define UAVAP_PITCHSTATESPACECASCADE_H


#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceControllerTarget.h"

class PitchStateSpaceCascade
{
public:
	PitchStateSpaceCascade(const std::vector<FloatingType> &a, const std::vector<FloatingType> &b, const std::vector<FloatingType> &k);

private:
	Eigen::Matrix<FloatingType, 5, 5> a_;
	Eigen::Matrix<FloatingType, 5, 2> b_;
	Eigen::Matrix<FloatingType, 2, 5> k_;
//	SensorData* sensorData_;
//	PitchStateSpaceControllerTarget* controllerTarget_;
//	Control::ControlEnvironment controlEnv_;
//
//	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
//	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> outputs_;
//
//	std::shared_ptr<Control::Constraint<>> rollConstraint_;
//	std::shared_ptr<Control::Constraint<>> pitchConstraint_;
//

//
//	double hardRollConstraint_;
//	double hardPitchConstraint_;
//	double beta_;
//	double rollTarget_;
};


#endif //UAVAP_PITCHSTATESPACECASCADE_H
