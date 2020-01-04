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
 * Metrics.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: simonyu
 */

#include <boost/property_tree/ptree.hpp>
#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"

template<class Group, typename Type>
void
mapMetricsValue(PropertyMapper<Configuration>& pm, const std::string& metrics, const std::string& metricsMember,
		Group& metricsGroup)
{
	auto metricsValueEnum = EnumMap<Type>::convert(metricsMember);

	if (metricsValueEnum != Type::INVALID)
	{
		auto insert = metricsGroup.insert(std::make_pair(metricsValueEnum, SteadyStateMetrics()));
		pm.add<double>(metrics, insert.first->second.tolerance, true);
	}
	else
	{
		APLOG_WARN << "Metrics: Invalid Metrics Member for " << metrics;
	}

	return;
}

void
SteadyStateMetrics::setToleranceTimeStamp(TimePoint time)
{
	toleranceTimeStamp = time;
}


void
Metrics::setToleranceTimeStamp(TimePoint time)
{
	for (auto& it : localPlanner)
	{
		it.second.setToleranceTimeStamp(time);
	}

	for (auto& it : controllerTarget)
	{
		it.second.setToleranceTimeStamp(time);
	}

	for (auto& it : pid)
	{
		it.second.setToleranceTimeStamp(time);
	}
}

void
Metrics::reset()
{
	for (auto& it : localPlanner)
	{
		it.second.reset();
	}

	for (auto& it : controllerTarget)
	{
		it.second.reset();
	}

	for (auto& it : pid)
	{
		it.second.reset();
	}
}

bool
Metrics::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	std::string metrics;
	std::string metricsGroup;
	std::string metricsMember;

	for (const auto& it : config)
	{
		metrics = it.first;
		std::istringstream ss(metrics);

		if (!std::getline(ss, metricsGroup, '/') || !std::getline(ss, metricsMember, '/'))
		{
			APLOG_WARN << "Metrics: Invalid Metrics: " << metrics;
		}

		auto metricsGroupEnum = EnumMap<MetricsGroup>::convert(metricsGroup);

		switch (metricsGroupEnum)
		{
		case MetricsGroup::LOCAL_PLANNER:
		{
			mapMetricsValue<LocalPlannerMetrics, LocalPlannerTargets>(pm, metrics, metricsMember,
					localPlanner);

			break;
		}
		case MetricsGroup::CONTROLLER_TARGETS:
		{
			mapMetricsValue<ControllerTargetMetrics, ControllerTargets>(pm, metrics, metricsMember,
					controllerTarget);

			break;
		}
		case MetricsGroup::PIDS:
		{
			mapMetricsValue<PIDMetrics, PIDs>(pm, metrics, metricsMember, pid);

			break;
		}
		case MetricsGroup::INVALID:
		{
			APLOG_WARN << "Metrics: Invalid Metrics Group for " << metrics;
			break;
		}
		default:
		{
			APLOG_WARN << "Metrics: Unknown Metrics Group for " << metrics;
		}
		}
	}

	degreeToRadian(*this);

	return pm.map();
}

bool
Metrics::isEmpty() const
{
	return localPlanner.empty() && controllerTarget.empty() && pid.empty();
}

void
degreeToRadian(Metrics& metrics)
{
	if (auto pair = findInMap(metrics.localPlanner, LocalPlannerTargets::HEADING))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.controllerTarget, ControllerTargets::CLIMB_ANGLE))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.controllerTarget, ControllerTargets::YAW_RATE))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::CLIMB_ANGLE))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::PITCH))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::PITCH_RATE))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::ROLL))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::ROLL_RATE))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::RUDDER))
		pair->second.tolerance *= M_PI / 180.0;

	if (auto pair = findInMap(metrics.pid, PIDs::YAW_RATE))
		pair->second.tolerance *= M_PI / 180.0;
}
