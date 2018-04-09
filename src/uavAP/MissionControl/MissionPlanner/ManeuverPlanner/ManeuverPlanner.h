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

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_H_
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/protobuf/messages/ManeuverPlanner.pb.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Scheduler/Event.h"
#include "uavAP/MissionControl/MissionPlanner/ControlOverride.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/ManeuverPlanner/ManeuverSet.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include <memory>
#include <unordered_map>

class IPC;
class IGlobalPlanner;
class IScheduler;
class SensorData;

/**
 * @brief   The ManeuverPlanner class is a mission planner that can accept
 *          maneuver overrides from the ground station. It's controller cascade
 *          has switches which can be switched from PID output to override output.
 */
class ManeuverPlanner : public IMissionPlanner, public IAggregatableObject, public IRunnableObject
{
public:

    ManeuverPlanner();

    static std::shared_ptr<ManeuverPlanner>
    create(const boost::property_tree::ptree& config);

    bool
    configure(const boost::property_tree::ptree& config);

    void
    notifyAggregationOnUpdate(Aggregator& agg) override;

    bool
    run(RunStage stage) override;

    /**
     * @brief   groundStationOverride is called whenever a new control override
     *          from the ground station is recieved
     * @param   controlOverride is the structure containing the override details
     *          specified from the ground station
     */
    void
    groundStationOverride(const ControlOverride& controlOverride);

    /**
     * @brief   maneuverSetRequest searches for a list of maneuvers with a name
     *          matching maneuver. If found, it begins to perform those maneuvers
     * @param   maneuver name of set of maneuvers to be performed
     */
    void
    maneuverSetRequest(const std::string& maneuver);

    /**
     * @brief   missionRequest searches for a list of waypoints with a name
     *          matching mission. If found, it sets the current mission to that
     *          set of waypoints and then begins to fly it.
     * @param   mission name of set of waypoints to fly
     */
    void
    missionRequest(const std::string& mission);

    /**
     * @brief   getSafetyRectangle returns the safety rectangle which aircraft
     *          is bounded to
     * @return  protobuf rectangle containing major side length, minor side
     *          length, major side orientation, and UTM ENU center that aircraft
     *          is bound to
     */
    Rectangle
    getSafetyRectangle() const;

    /**
     * @brief   getControlOverride returns current control override settings
     * @return  ControlOverride which specifies current override settings
     */
    ControlOverride
    getControlOverride() const;

private:

    /**
     * @brief   publishMission notifies the global planner of the new mission to
     *          fly
     */
    void
    publishMission();

    /**
     * @brief   onSensorData is called every time sensor data is recieved. This
     *          function checks to see if the aircraft has left safety bounds
     *          and if so, disables the overrride
     * @param   data sensor data containing current override position
     */
    void
    onSensorData(const SensorData& data);

    /**
     * @brief   startManeuver starts a single maneuver. It sets the targets for
     *          roll, pitch, and velocity, then schedules the next maneuver.
     */
    void
    startManeuver();

    /**
     * @brief   nextManeuver increments the iterators for roll, pitch, and
     *          velocity targets, and then starts the maneuver. If the iterators
     *          are all at their ends, then it ends the maneuver.
     */
    void
    nextManeuver();

    /**
     * @brief   stopManeuver disables all overrides and allows the aircraft to
     *          resume it's mission.
     */
    void
    stopManeuver();

    ManeuverPlannerParams params_;

    Publisher overrideTargetPublisher_;
    Publisher overrideActivationPublisher_;

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
    ControlOverride lastManualOverride_;

    ObjectHandle<IPC> ipc_;
    ObjectHandle<IGlobalPlanner> globalPlanner_;
    ObjectHandle<IScheduler> scheduler_;
};



#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_H_ */
