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
 * CustomPlanner.h
 *
 *  Created on: Nov 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CUSTOMPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CUSTOMPLANNER_H_

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include <memory>
#include <unordered_map>

class IPC;
class IGlobalPlanner;
class IScheduler;
class SensorData;

class CustomPlanner : public IMissionPlanner, public IAggregatableObject, public IRunnableObject
{
public:

	CustomPlanner();

	static std::shared_ptr<CustomPlanner>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	groundStationOverride(const ManeuverOverride& manOverride);

	void
	maneuverSetRequest(const std::string& maneuver);

	void
	missionRequest(const std::string& mission);

	Rectangle
	getSafetyRectangle() const;

private:

	void
	publishMission();

	void
	onSensorData(const SensorData& data);

	void
	startManeuver();

	void
	nextManeuver();

	void
	stopManeuver();

	ManeuverPlannerParams params_;

	Publisher maneuverTargetPublisher_;
	Publisher maneuverActivationPublisher_;

	Subscription sensorDataSubscription_;

	using ManeuverMap = std::unordered_map<std::string, ManeuverSet>;
	ManeuverMap maneuverSetMap_;
	ManeuverMap::const_iterator currentManeuverSet_;

	using MissionMap = std::unordered_map<std::string, Mission>;
	MissionMap missionMap_;
	MissionMap::const_iterator currentMission_;

	using OverrideTargetIterator = std::vector<double>::const_iterator;
	OverrideTargetIterator rollIterator_;
	OverrideTargetIterator pitchIterator_;
	OverrideTargetIterator velocityIterator_;
	Event nextManeuverEvent_;

	bool overrideActive_;
	bool overrideRestart_;
	ManeuverOverride lastManualOverride_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IGlobalPlanner> globalPlanner_;
	ObjectHandle<IScheduler> scheduler_;
};


#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_CUSTOMPLANNER_H_ */
