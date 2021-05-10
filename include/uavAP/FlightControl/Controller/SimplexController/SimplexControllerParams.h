//
// Created by seedship on 4/30/21.
//

#ifndef UAVAP_SIMPLEXCONTROLLERPARAMS_H
#define UAVAP_SIMPLEXCONTROLLERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceParams.h"

struct SimplexControllerParams
{
	// Pitch State Space Params
	Parameter<PitchStateSpaceParams> controllerParams = {{}, "controller_params", true};

	// Simplex Params
	Parameter<Eigen::Matrix<FloatingType, 7, 7, Eigen::DontAlign>> p = {{}, "p", true};
	Parameter<Eigen::Matrix<FloatingType, 7, 7, Eigen::DontAlign>> a = {{}, "a", true};
	Parameter<Eigen::Matrix<FloatingType, 7, 2, Eigen::DontAlign>> b = {{}, "b", true};
	Parameter<VectorN<7>> kPitch = {{}, "k_pitch", true};
	Parameter<FloatingType> safetyAlt = {2000, "safety_altitude", true};
	Parameter<Angle<FloatingType>> maxPitchTarget = {Angle<FloatingType>(15), "max_pitch_target", true};
	Parameter<Angle<FloatingType>> minPitchTarget = {Angle<FloatingType>(-15), "min_pitch_target", true};

	Parameter<FloatingType> controllerPeriodMS = {100, "controller_period_ms", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & controllerParams;
		c & p;
		c & a;
		c & b;
		c & kPitch;
		c & safetyAlt;
		c & maxPitchTarget;
		c & minPitchTarget;
		c & controllerPeriodMS;
	}
};

#endif //UAVAP_SIMPLEXCONTROLLERPARAMS_H
