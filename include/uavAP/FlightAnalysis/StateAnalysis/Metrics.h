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
 * Metrics.h
 *
 *  Created on: Aug 14, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_STATEANALYSIS_METRICS_H_
#define UAVAP_FLIGHTANALYSIS_STATEANALYSIS_METRICS_H_

#include <utility>

#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/FlightControl/LocalPlanner/LocalPlannerTargets.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"

enum class MetricsGroup
{
	INVALID, LOCAL_PLANNER, CONTROLLER_TARGETS, PIDS, NUM_METRICS
};

ENUMMAP_INIT(MetricsGroup, { {MetricsGroup::LOCAL_PLANNER, "local_planner"},
		{MetricsGroup::CONTROLLER_TARGETS, "controller_targets"}, {MetricsGroup::PIDS, "pids"} });

using InspectingMetricsPair = std::pair<int, int>;

struct SteadyStateMetrics
{
	TimePoint toleranceTimeStamp;
	double target;
	double lastTarget;
	double value;
	double lastValue;
	double tolerance;
	double overshoot;
	double riseTime;
	double settlingTime;
	bool inTolerance;
	bool inSteadyState;
	bool isReset;
	bool crossedTarget;
	bool foundLastTarget;
	bool foundRiseTime;
	bool foundSettlingTime;

	SteadyStateMetrics() :
			toleranceTimeStamp(), target(0), value(0), tolerance(0)
	{
		SteadyStateMetrics::reset();
	}


	void
	setToleranceTimeStamp(TimePoint time);

	void
	reset()
	{
		lastTarget = target;
		target = 0;
		lastValue = value;
		value = 0;
		overshoot = 0;
		riseTime = 0;
		settlingTime = 0;
		inTolerance = false;
		inSteadyState = false;
		isReset = true;
		crossedTarget = false;
		foundLastTarget = false;
		foundRiseTime = false;
		foundSettlingTime = false;
	}
};

struct Metrics
{
	using LocalPlannerMetrics = std::map<LocalPlannerTargets, SteadyStateMetrics>;
	using ControllerTargetMetrics = std::map<ControllerTargets, SteadyStateMetrics>;
	using PIDMetrics = std::map<PIDs, SteadyStateMetrics>;

	LocalPlannerMetrics localPlanner;
	ControllerTargetMetrics controllerTarget;
	PIDMetrics pid;

	bool
	configure(const Configuration& config);

	bool
	isEmpty() const;

	void
	setToleranceTimeStamp(TimePoint time);

	void
	reset();
};

void
degreeToRadian(Metrics& metrics);

namespace dp
{

template<class Archive, typename Type>
inline void
serialize(Archive& ar, SteadyStateMetrics& t)
{
	ar & t.toleranceTimeStamp;
	ar & t.target;
	ar & t.lastTarget;
	ar & t.value;
	ar & t.lastValue;
	ar & t.tolerance;
	ar & t.overshoot;
	ar & t.riseTime;
	ar & t.settlingTime;
	ar & t.inTolerance;
	ar & t.inSteadyState;
	ar & t.isReset;
	ar & t.crossedTarget;
	ar & t.foundLastTarget;
	ar & t.foundRiseTime;
	ar & t.foundSettlingTime;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Metrics& t)
{
	ar & t.localPlanner;
	ar & t.controllerTarget;
	ar & t.pid;
}

} /* dp */

#endif /* UAVAP_FLIGHTANALYSIS_STATEANALYSIS_METRICS_H_ */
