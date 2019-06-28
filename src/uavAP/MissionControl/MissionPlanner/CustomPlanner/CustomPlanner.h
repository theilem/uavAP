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
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_H_

#include <memory>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"

class IPC;
class IGlobalPlanner;
class IScheduler;

struct SensorData;

/**
 * @brief   The CustomPlanner class is a mission planner that can accept
 *          maneuver overrides from the ground station. It's controller cascade
 *          has switches which can be switched from PID output to override output.
 */
class CustomPlanner : public IMissionPlanner, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "custom";

	CustomPlanner();

	bool
	configure(const boost::property_tree::ptree& config);

	ADD_CREATE_WITH_CONFIG(CustomPlanner)

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	/**
	 * @brief   missionRequest searches for a list of waypoints with a name
	 *          matching mission. If found, it sets the current mission to that
	 *          set of waypoints and then begins to fly it.
	 * @param   mission name of set of waypoints to fly
	 */
	void
	missionRequest(const std::string& mission) override;

private:

	using MissionMap = std::unordered_map<std::string, Mission>;

	/**
	 * @brief   publishMission notifies the global planner of the new mission to
	 *          fly
	 */
	void
	publishMission();

	/**
	 * @brief   onSensorData is called every time sensor data is received. This
	 *          function checks to see if the aircraft has left safety bounds
	 *          and if so, disables the override
	 * @param   data sensor data containing current override position
	 */
	void
	onSensorData(const SensorData& data);

	double defaultVelocity_;
	double defaultAltitude_;
	bool useApproach_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IGlobalPlanner> globalPlanner_;

	Subscription sensorDataSubscription_;

	MissionMap missionMap_;
	MissionMap::const_iterator currentMission_;

	Vector3 currentPosition_;
	Vector3 currentDirection_;
	std::mutex positionMutex_;
};

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_H_ */
