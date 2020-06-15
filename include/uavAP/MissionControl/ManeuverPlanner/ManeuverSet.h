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
	std::map<ControllerOutputs, bool> controllerOutputOverrideMap;
	ControllerOutputsOverrides controllerOutputOverrideType = ControllerOutputsOverrides::INVALID;
	bool controllerOutputOverrideFlag = false;
	bool analyzeManeuver = false;
	bool analyzeTrim = false;

	bool
	configure(const Configuration& config)
	{
		PropertyMapper<Configuration> pm(config);
		Configuration conditionTree;
		Configuration controllerOutputOverrideTree;
		ConditionFactory factory;

		pm.add<Override>("override", override, true);

		if (pm.add("override_controller_outputs", controllerOutputOverrideTree, false))
		{
			PropertyMapper<Configuration> controllerOutputOverridePm(controllerOutputOverrideTree);

			controllerOutputOverridePm.addEnum<ControllerOutputsOverrides>("type",
					controllerOutputOverrideType, true);

			for (auto& overrideIt : controllerOutputOverrideTree)
			{
				if (overrideIt.first == "enable")
				{
					controllerOutputOverridePm.add<bool>(overrideIt.first,
							controllerOutputOverrideFlag, true);
				}
				else
				{
					if (!controllerOutputOverrideFlag)
					{
						continue;
					}

					bool overrideOutput = false;

					if (controllerOutputOverridePm.add<bool>(overrideIt.first, overrideOutput,
							true))
					{
						auto controllerOutputsEnum = EnumMap<ControllerOutputs>::convert(
								overrideIt.first);
						controllerOutputOverrideMap.insert(
								std::make_pair(controllerOutputsEnum, overrideOutput));
					}
				}
			}
		}

		pm.add("condition", conditionTree, true);
		pm.add<AdvancedControl>("advanced_control", advancedControl, false);
		pm.add<bool>("analyze_maneuver", analyzeManeuver, false);
		pm.add<bool>("analyze_trim", analyzeTrim, false);

		condition = ConditionFactory::create(conditionTree);

		return pm.map();
	}
};

using ManeuverSet = std::vector<Maneuver>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_ */
