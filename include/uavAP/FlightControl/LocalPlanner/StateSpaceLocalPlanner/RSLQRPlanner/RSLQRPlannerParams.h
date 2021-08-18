//
// Created by seedship on 7/25/21.
//

#ifndef UAVAP_RSLQRPLANNERPARAMS_H
#define UAVAP_RSLQRPLANNERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/Utilities/LinearAlgebra.h>

#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/StateSpaceUtils/GainScheduler.h"

struct RSLQRPlannerParams
{
//	Parameter<Eigen::Matrix<FloatingType, 2, 16, Eigen::DontAlign>> k = {{}, "k", true};
	Parameter<GainSchedulingParams<2, 16>> k_sched = {{}, "k_sched", true};

	Parameter<FloatingType> kConvergence = {1.0, "k_convergence", true};
	Parameter<FloatingType> safetyVelocity = {55, "safety_velocity", true};

	Parameter<Control::IntegratorParams> pitchParams = {Control::IntegratorParams(), "pitch_target_params_rad", true};
	Parameter<Control::IntegratorParams> rollParams = {Control::IntegratorParams(), "roll_target_params_rad", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & k_sched;
		c & kConvergence;
		c & safetyVelocity;
		c & pitchParams;
		c & rollParams;
	}
};

#endif //UAVAP_RSLQRPLANNERPARAMS_H
