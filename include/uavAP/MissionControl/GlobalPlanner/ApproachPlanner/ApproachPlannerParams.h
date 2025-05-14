/*
 * ApproachPlannerParams.h
 *
 *  Created on: May 14, 2025
 *      Author: acmeighan, jjponnia
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNERPARAMS_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNERPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/Utilities/LinearAlgebra.h>

struct ApproachPlannerParams
{
	Parameter<FloatingType> orbitRadius = {50.0, "orbit_radius", false};
	Parameter<FloatingType> tau = {0.5, "tau", false};
	Parameter<bool> smoothenZ = {true, "smoothen_z", false};
	Parameter<bool> naturalSplines = {false, "natural", false};
	Parameter<uint8_t> inclusionLength = {0, "inclusion_length", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & orbitRadius;
		c & tau;
		c & smoothenZ;
		c & naturalSplines;
		c & inclusionLength;
	}
};


#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNERPARAMS_H_ */
