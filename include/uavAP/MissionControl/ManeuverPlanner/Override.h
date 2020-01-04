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
 * ControlOverride.h
 *
 *  Created on: Nov 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_OVERRIDE_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_OVERRIDE_H_


#include <cpsCore/Configuration/ConfigurableObject.hpp>

#include "uavAP/FlightControl/Controller/ControllerConstraint.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/LocalPlanner/LocalPlannerTargets.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

enum class CustomOverrideIDs
{
	INVALID, CUSTOM_1, CUSTOM_2, CUSTOM_3, CUSTOM_4, NUM_CUSTOM
};

enum class OverrideGroup
{
	INVALID,
	LOCAL_PLANNER,
	CONTROLLER_TARGETS,
	PIDS,
	CONTROLLER_OUTPUTS,
	CONTROLLER_OUTPUTS_WAVEFORMS,
	CONTROLLER_CONSTRAINTS,
	CUSTOM,
	NUM_GROUP
};

ENUMMAP_INIT(CustomOverrideIDs, {
	{ CustomOverrideIDs::CUSTOM_1, "custom_1" },
	{ CustomOverrideIDs::CUSTOM_2, "custom_2" }, { CustomOverrideIDs::CUSTOM_3, "custom_3" },
	{ CustomOverrideIDs::CUSTOM_4, "custom_4" }
});

ENUMMAP_INIT(OverrideGroup, {
	{ OverrideGroup::LOCAL_PLANNER, "local_planner" },
	{ OverrideGroup::CONTROLLER_TARGETS, "controller_targets" }, { OverrideGroup::PIDS, "pids" },
	{ OverrideGroup::CONTROLLER_OUTPUTS, "controller_outputs" },
	{ OverrideGroup::CONTROLLER_OUTPUTS_WAVEFORMS, "controller_outputs_waveforms" },
	{ OverrideGroup::CONTROLLER_CONSTRAINTS, "controller_constraints" }, {
		OverrideGroup::CUSTOM,
		"custom"
	}
});

struct Override
{
	using LocalPlannerOverrides = std::map<LocalPlannerTargets, double>;
	using ControllerTargetOverrides = std::map<ControllerTargets, double>;
	using PIDOverrides = std::map<PIDs, double>;
	using ControllerOutputOverrides = std::map<ControllerOutputs, double>;
	using ControllerOutputWaveformOverrides = std::map<ControllerOutputsWaveforms, Waveforms>;
	using ControllerConstraintOverrides = std::map<ControllerConstraints, double>;
	using CustomOverrides = std::map<CustomOverrideIDs, double>;

	LocalPlannerOverrides localPlanner;
	ControllerTargetOverrides controllerTarget;
	PIDOverrides pid;
	ControllerOutputOverrides output;
	ControllerOutputWaveformOverrides waveform;
	ControllerConstraintOverrides constraint;
	CustomOverrides custom;
	double wavelength;
	double phase;

	bool
	configure(const Configuration& config);

	bool
	isEmpty() const;
};

template<class Group, typename Type>
void
mapOverrideValue(PropertyMapper<Configuration>& pm, const std::string& override,
				 const std::string& overrideMember, Group& overrideGroup)
{
	auto overrideValueEnum = EnumMap<Type>::convert(overrideMember);

	if (overrideValueEnum != Type::INVALID)
	{
		auto insert = overrideGroup.insert(std::make_pair(overrideValueEnum, 0));
		pm.add<double>(override, insert.first->second, true);
	}
	else
	{
		CPSLOG_WARN << "Override: Invalid Override Member for " << override;
	}

	return;
}

template<class Group, typename Type, typename Enum>
void
mapOverrideEnum(PropertyMapper<Configuration>& pm, const std::string& override,
				const std::string& overrideMember, Group& overrideGroup)
{
	auto overrideValueEnum = EnumMap<Type>::convert(overrideMember);

	if (overrideValueEnum != Type::INVALID)
	{
		auto insert = overrideGroup.insert(std::make_pair(overrideValueEnum, Enum::INVALID));
		pm.addEnum<Enum>(override, insert.first->second, true);
	}
	else
	{
		CPSLOG_WARN << "Override: Invalid Override Member for " << override;
	}

	return;
}

void
degreeToRadian(Override& override);

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Override& t)
{
	ar & t.localPlanner;
	ar & t.controllerTarget;
	ar & t.pid;
	ar & t.output;
	ar & t.waveform;
	ar & t.constraint;
	ar & t.custom;
	ar & t.wavelength;
	ar & t.phase;
}
} /* dp */

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_OVERRIDE_H_ */
