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
 * ManeuverPlanner.h
 *
 *  Created on: Aug 12, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNER_H_
#define UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNER_H_

#include <string>
#include <unordered_map>

#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/protobuf/messages/ManeuverPlanner.pb.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightAnalysis/ManeuverAnalysis/ManeuverAnalysisStatus.h"
#include "uavAP/MissionControl/ManeuverPlanner/ManeuverSet.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"

class IPC;
class ConditionManager;
class RectanguloidCondition;

class ManeuverPlanner: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "maneuver_planner";

	ManeuverPlanner();

	static std::shared_ptr<ManeuverPlanner>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	setManualOverride(const Override& override);

	void
	setManeuverOverride(const std::string& maneuverSet);

	void
	interruptOverride();

	void
	resumeOverride();

	Override
	getOverride() const;

	unsigned int
	getOverrideNr() const;

	Rectanguloid
	getSafetyBounds() const;

private:

	void
	setOverride(const std::string& maneuverSet, const bool& manualActive,
			const bool& maneuverActive);

	void
	startOverride();

	void
	stopOverride();

	void
	nextManeuverOverride();

	void
	activateManeuverOverride(const ICondition::ConditionTrigger& conditionTrigger);

	void
	deactivateManeuverOverride();

	void
	safetyTrigger(int trigger);

	using ManeuverSetMap = std::unordered_map<std::string, ManeuverSet>;

	ManeuverPlannerParams params_;
	ManeuverAnalysisStatus analysis_;

	Override override_;
	mutable std::mutex overrideMutex_;

	ManeuverSetMap maneuverSetMap_;
	ManeuverSetMap::const_iterator currentManeuverSet_;
	boost::optional<ManeuverSet::const_iterator> currentManeuver_;

	std::string maneuverSet_;
	bool overrideInterrupted_;
	bool manualActive_;
	bool maneuverActive_;

	Override lastManualOverride_;
	std::string lastManeuverSet_;
	boost::optional<ManeuverSet::const_iterator> lastManeuver_;
	bool lastManualActive_;
	bool lastManeuverActive_;

	bool manualRestart_;
	bool maneuverRestart_;

	unsigned int overrideSeqNr_;
	mutable std::mutex overrideSeqNrMutex_;

	std::shared_ptr<RectanguloidCondition> safetyCondition_;

	Publisher overridePublisher_;
	Publisher maneuverAnalysisPublisher_;
	Subscription sensorDataSubscription_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<ConditionManager> conditionManager_;
};

#endif /* UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNER_H_ */
