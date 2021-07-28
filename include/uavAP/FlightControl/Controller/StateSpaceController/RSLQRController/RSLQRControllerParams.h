//
// Created by seedship on 7/23/21.
//

#ifndef UAVAP_RSLQRCONTROLLERPARAMS_H
#define UAVAP_RSLQRCONTROLLERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

#include <cpsCore/Utilities/LinearAlgebra.h>

struct RSLQRControllerParams
{
	Parameter<Eigen::Matrix<FloatingType, 4, 12, Eigen::DontAlign>> k = {{}, "k", true};

	Parameter<Control::IntegratorParams> elevatorParams = {Control::IntegratorParams(), "elevator_params", true};
	Parameter<Control::IntegratorParams> throttleParams = {Control::IntegratorParams(), "throttle_params", true};
	Parameter<Control::IntegratorParams> aileronParams = {Control::IntegratorParams(), "aileron_params", true};
	Parameter<Control::IntegratorParams> rudderParams = {Control::IntegratorParams(), "rudder_params", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & k;

		c & elevatorParams;
		c & throttleParams;
		c & aileronParams;
		c & rudderParams;
	}
};

#endif //UAVAP_RSLQRCONTROLLERPARAMS_H
