////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * Maneuver.h
 *
 *  Created on: Sep 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_

#include <vector>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/MissionControl/ConditionManager/ConditionFactory.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"

struct Maneuver
{
	Override override;
	AdvancedControl advancedControl;
	std::shared_ptr<ICondition> condition;
	bool analysis;

	bool
	configure(const Configuration& config)
	{
		PropertyMapper pm(config);
		boost::property_tree::ptree conditionTree;
		static ConditionFactory factory;

		pm.add<Override>("override", override, true);
		pm.add("condition", conditionTree, true);
		pm.add<AdvancedControl>("advanced_control", advancedControl, false);
		pm.add<bool>("analysis", analysis, false);

		condition = factory.create(conditionTree);

		return pm.map();
	}
};

using ManeuverSet = std::vector<Maneuver>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_ */
