/*
 * ManeuverRatePIDControllerParams.h
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLERPARAMS_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLERPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>

struct ManeuverRatePIDControllerParams
{

	Parameter<unsigned> period = {0, "period", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & period;
	}
};


#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLERPARAMS_H_ */
