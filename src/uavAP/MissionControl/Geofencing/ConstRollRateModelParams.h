/*
 * ConstRollRateModelParams.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODELPARAMS_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODELPARAMS_H_
#include <uavAP/Core/Angle.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/Parameter.h>

struct ConstRollRateModelParams
{
	Parameter<Angle<FloatingType>> rollMax = {Angle<FloatingType>(45.0), "roll_max", true};
	Parameter<Angle<FloatingType>> rollRate = {Angle<FloatingType>(30.0), "roll_rate", true};
	Parameter<unsigned> precision = {32, "precision", false};
	Parameter<FloatingType> g = {9.81, "g", false};
	Parameter<bool> useWind = {false, "use_wind", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & rollMax;
		c & rollRate;
		c & precision;
		c & g;
		c & useWind;
	}

};



#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODELPARAMS_H_ */
