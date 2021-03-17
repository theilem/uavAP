/*
 * ManeuverPlannerParams.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_
#define UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/ManeuverPlanner/Maneuver.h"

struct ManeuverPlannerParams
{
	Parameter<std::map<std::string, ManeuverSet>> maneuverSets = {{{}}, "maneuver_sets", true};
	Parameter<int> period = {10, "period", true};
	Parameter<std::string> logPath = {"/tmp/log/", "log_path", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & maneuverSets;
		c & period;
		c & logPath;
	}
};


#endif /* UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_ */
