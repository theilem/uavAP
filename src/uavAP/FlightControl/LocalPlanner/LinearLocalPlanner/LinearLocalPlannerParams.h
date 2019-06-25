/*
 * LinearLocalPlannerParams.h
 *
 *  Created on: Jun 23, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERPARAMS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERPARAMS_H_
#include "uavAP/Core/PropertyMapper/Parameter.h"
#include "uavAP/Core/LinearAlgebra.h"


struct LinearLocalPlannerParams
{
	Parameter<FloatingType> kAltitude = {1.0, "k_altitude", true};
	Parameter<FloatingType> kHeading = {1.0, "k_heading", true};
	Parameter<FloatingType> kYawrate = {1.0, "k_yawrate", true};
	Parameter<int> period = {0, "period", false};

	template <class Configurator>
	inline void
	configure(Configurator& c)
	{
		c & kAltitude;
		c & kHeading;
		c & kYawrate;
		c & period;
	}
};


#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERPARAMS_H_ */
