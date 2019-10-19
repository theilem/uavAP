/*
 * GeofencingParams.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCINGPARAMS_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCINGPARAMS_H_
#include <uavAP/Core/Angle.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/Parameter.h>


struct GeofencingParams
{

	Parameter<Angle<FloatingType>> rollMax = {Angle<FloatingType>(45.0), "roll_max", true};
	Parameter<FloatingType> evaluationThreshold = {150.0, "evaluation_threshold", true};
	Parameter<FloatingType> distanceThreshold = {150.0, "distance_threshold", true};
	Parameter<unsigned> period = {50, "period", true};
	Parameter<Vector3> windEstimate = {Vector3(0,0,0), "wind_estimate", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & rollMax;
		c & evaluationThreshold;
		c & distanceThreshold;
		c & period;
		c & windEstimate;
	}

};


#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCINGPARAMS_H_ */
