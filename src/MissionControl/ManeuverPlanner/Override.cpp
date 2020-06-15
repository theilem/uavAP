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
 * Override.cpp
 *
 *  Created on: Aug 4, 2018
 *      Author: mircot
 */

#include <sstream>
#include <string>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/MissionControl/ManeuverPlanner/Override.h"

bool
Override::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	std::string override;
	std::string overrideGroup;
	std::string overrideMember;

	for (const auto& it : config)
	{
		override = it.first;

		if (override == "wavelength")
		{
			pm.add<double>(override, wavelength, false);
			continue;
		}
		else if (override == "phase")
		{
			pm.add<double>(override, phase, false);
			continue;
		}

		std::istringstream ss(override);

		if (!std::getline(ss, overrideGroup, '/') || !std::getline(ss, overrideMember, '/'))
		{
			CPSLOG_WARN << "Override: Invalid Override: " << override;
		}

		auto overrideGroupEnum = EnumMap<OverrideGroup>::convert(overrideGroup);

		switch (overrideGroupEnum)
		{
		case OverrideGroup::LOCAL_PLANNER:
		{
			mapOverrideValue<LocalPlannerOverrides, LocalPlannerTargets>(pm, override,
					overrideMember, localPlanner);

			break;
		}
		case OverrideGroup::CONTROLLER_TARGETS:
		{
			mapOverrideValue<ControllerTargetOverrides, ControllerTargets>(pm, override,
					overrideMember, controllerTarget);

			break;
		}
		case OverrideGroup::PIDS:
		{
			mapOverrideValue<PIDOverrides, PIDs>(pm, override, overrideMember, pid);

			break;
		}
		case OverrideGroup::CONTROLLER_OUTPUTS:
		{
			mapOverrideValue<ControllerOutputOverrides, ControllerOutputs>(pm, override,
					overrideMember, output);

			break;
		}
		case OverrideGroup::CONTROLLER_OUTPUTS_WAVEFORMS:
		{
			mapOverrideEnum<ControllerOutputWaveformOverrides, ControllerOutputsWaveforms, Waveforms>(
					pm, override, overrideMember, waveform);

			break;
		}
		case OverrideGroup::CONTROLLER_CONSTRAINTS:
		{
			mapOverrideValue<ControllerConstraintOverrides, ControllerConstraints>(pm, override,
					overrideMember, constraint);

			break;
		}
		case OverrideGroup::CUSTOM:
		{
			mapOverrideValue<CustomOverrides, CustomOverrideIDs>(pm, override, overrideMember,
					custom);

			break;
		}
		case OverrideGroup::INVALID:
		{
			CPSLOG_WARN << "Override: Invalid Override Group for " << override;
			break;
		}
		default:
		{
			CPSLOG_WARN << "Override: Unknown Override Group for " << override;
		}
		}
	}

	degreeToRadian(*this);

	return pm.map();
}

bool
Override::isEmpty() const
{
	return localPlanner.empty() && controllerTarget.empty() && pid.empty() && output.empty()
			&& waveform.empty() && constraint.empty() && custom.empty();
}

void
degreeToRadian(Override& override)
{
	if (auto pair = findInMap(override.localPlanner, LocalPlannerTargets::HEADING))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.controllerTarget, ControllerTargets::CLIMB_ANGLE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.controllerTarget, ControllerTargets::YAW_RATE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::CLIMB_ANGLE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::PITCH))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::PITCH_RATE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::ROLL))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::ROLL_RATE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::RUDDER))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.pid, PIDs::YAW_RATE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.constraint, ControllerConstraints::ROLL))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.constraint, ControllerConstraints::ROLL_RATE))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.constraint, ControllerConstraints::PITCH))
		pair->second *= M_PI / 180.0;

	if (auto pair = findInMap(override.constraint, ControllerConstraints::PITCH_RATE))
		pair->second *= M_PI / 180.0;
}
