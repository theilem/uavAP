//
// Created by seedship on 4/30/21.
//

#ifndef UAVAP_SIMPLEXCONTROLLERPARAMS_H
#define UAVAP_SIMPLEXCONTROLLERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/StateSpaceAltitudePlannerParams.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceParams.h"

struct SimplexControllerParams
{

	static constexpr const char* SIMPLEX_RECOVERYMODE_TIME = "time";
	static constexpr const char* SIMPLEX_RECOVERYMODE_VALUE = "value";

	// Pitch State Space Params
	Parameter<PitchStateSpaceParams> controllerParams = {{}, "controller_params", true};

	// Simplex Params
	Parameter<Eigen::Matrix<FloatingType, 5, 5, Eigen::DontAlign>> a = {{}, "a", true};
	Parameter<Eigen::Matrix<FloatingType, 5, 2, Eigen::DontAlign>> b = {{}, "b", true};
	Parameter<Eigen::Matrix<FloatingType, 5, 5, Eigen::DontAlign>> p = {{}, "p", true};
	Parameter<Eigen::Matrix<FloatingType, 2, 5, Eigen::DontAlign>> k = {{}, "k", true};

	Parameter<FloatingType> safetyAlt = {2438, "safety_altitude", true};
	Parameter<Angle<FloatingType>> safetyMaxPitchTarget = {Angle<FloatingType>(15), "max_pitch_target", true};
	Parameter<Angle<FloatingType>> safetyMinPitchTarget = {Angle<FloatingType>(-15), "min_pitch_target", true};
	Parameter<FloatingType> controllerFrequency = {100, "controller_frequency", true};

	Parameter<std::string> recoveryMode = {SIMPLEX_RECOVERYMODE_VALUE, "recovery_mode", true};
	// If recovery mode set to "time"
	Parameter<int> recoveryTime = {20, "recovery_time", false};
	// If recovery mode set to "value"
	Parameter<FloatingType> recoveryValue = {0.1, "recovery_value", false};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & controllerParams;
		c & a;
		c & b;
		c & p;
		c & k;
		c & safetyAlt;
		c & safetyMaxPitchTarget;
		c & safetyMinPitchTarget;
		c & controllerFrequency;
		c & recoveryMode;
		c & recoveryTime;
		c & recoveryValue;
	}
};

#endif //UAVAP_SIMPLEXCONTROLLERPARAMS_H
