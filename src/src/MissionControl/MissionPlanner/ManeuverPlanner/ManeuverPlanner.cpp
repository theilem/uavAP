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
 * ManeuverPlanner.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/ManeuverPlanner/ManeuverPlanner.h"
#include "uavAP/Core/IPC/IPC.h"
#include <memory>

ManeuverPlanner::ManeuverPlanner() :
    overrideActive_(false),
    overrideRestart_(false)
{
}

std::shared_ptr<ManeuverPlanner>
ManeuverPlanner::create(const boost::property_tree::ptree& config)
{
    auto planner = std::make_shared<ManeuverPlanner>();
    planner->configure(config);

    return planner;
}

bool
ManeuverPlanner::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper pm(config);
    pm.configure(params_, true);

    boost::property_tree::ptree maneuvers;
    pm.add("maneuvers", maneuvers, false);

    for (auto& it : maneuvers)
    {
        ManeuverSet man;
        man.configure(it.second);
        maneuverSetMap_.insert(std::make_pair(it.first, man));
    }
    currentManeuverSet_ = maneuverSetMap_.end();

    if (!params_.has_safety_rectangle() || !params_.safety_rectangle().has_center())
    {
        APLOG_ERROR << "Parameters not set for Maneuver center";
        return false;
    }

    Mission defaultMission;
    Waypoint centerWP(toVector(params_.safety_rectangle().center()), params_.return_velocity());
    defaultMission.waypoints.push_back(centerWP);
    missionMap_.insert(std::make_pair("default", defaultMission));

    boost::property_tree::ptree missions;
    pm.add("missions", missions, false);

    for (auto& it : missions)
    {
        Mission mis;
        mis.configure(it.second);
        missionMap_.insert(std::make_pair(it.first, mis));
    }
    currentMission_ = missionMap_.find("default");

    return pm.map();
}

void
ManeuverPlanner::notifyAggregationOnUpdate(Aggregator& agg)
{
    ipc_.setFromAggregationIfNotSet(agg);
    globalPlanner_.setFromAggregationIfNotSet(agg);
    scheduler_.setFromAggregationIfNotSet(agg);
}

bool
ManeuverPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
    {
        if (!ipc_.isSet())
        {
            APLOG_ERROR << "ManeuverPlanner: ipc missing.";
            return true;
        }
        if (!scheduler_.isSet())
        {
            APLOG_ERROR << "SimpleMissionPlanner: Scheduler missing.";
            return true;
        }
        if (!globalPlanner_.isSet())
        {
            APLOG_ERROR << "SimpleMissionPlanner: Global Planner missing.";
            return true;
        }

        auto ipc = ipc_.get();
        overrideTargetPublisher_ = ipc->publishOnSharedMemory<OverrideTarget>("override_target");
        overrideActivationPublisher_ = ipc->publishOnSharedMemory<OverrideActivation>(
                                           "override_activation");
        break;
    }
    case RunStage::NORMAL:
    {
        auto ipc = ipc_.get();
        sensorDataSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
                                  std::bind(&ManeuverPlanner::onSensorData, this, std::placeholders::_1));

        if (!sensorDataSubscription_.connected())
        {
            APLOG_ERROR << "Sensor data missing.";
            return true;
        }

        auto scheduler = scheduler_.get();
        scheduler->schedule(std::bind(&ManeuverPlanner::publishMission, this), Milliseconds(0));
        break;
    }
    default:
        break;
    }
    return false;
}

void
ManeuverPlanner::publishMission()
{
    if (currentMission_ == missionMap_.end())
    {
        APLOG_ERROR << "No mission selected. Cannot publish.";
        return;
    }

    auto gp = globalPlanner_.get();
    if (!gp)
    {
        APLOG_ERROR << "Cannot set mission. Global Planner missing.";
        return;
    }

    APLOG_DEBUG << "Start Mission: " << currentMission_->first;
    gp->setMission(currentMission_->second);

}

void
ManeuverPlanner::groundStationOverride(const ControlOverride& manOverride)
{
    lastManualOverride_ = manOverride;
    if (manOverride.overrideManeuverPlanner)
    {
        overrideTargetPublisher_.publish(manOverride.target);
        overrideActivationPublisher_.publish(manOverride.activation);
        overrideActive_ = manOverride.activation.activate;
    }
    APLOG_DEBUG << "Manual override received";
    overrideRestart_ = false;
    nextManeuverEvent_.cancel();
}

void
ManeuverPlanner::onSensorData(const SensorData& data)
{
    if (!params_.has_safety_rectangle() || !params_.safety_rectangle().has_center())
    {
        APLOG_ERROR << "Parameters not set for Maneuver center";
        return;
    }
    auto rect = params_.safety_rectangle();
    auto position = data.position;

    Vector3 diffCenter = position - toVector(rect.center());
    Vector2 diffOriented = rotate2Drad(diffCenter.head(2),
                                       - rect.major_side_orientation() * M_PI / 180); // Rotate into other frame -> *(-1)
    bool isInside = (std::abs(diffOriented.x()) < rect.major_side_length() / 2)
                    && (std::abs(diffOriented.y()) < rect.minor_side_length() / 2)
                    && (std::abs(diffCenter.z()) < params_.safety_height() / 2);

    if ((!isInside || !data.autopilotActive) && overrideActive_)
    {
        APLOG_WARN << "Out of bounds. End override";
        lastManualOverride_.activation.activate = false;
        overrideActivationPublisher_.publish(lastManualOverride_.activation);
        overrideActive_ = false;
        nextManeuverEvent_.cancel();
    }

    if (!overrideActive_ && overrideRestart_)
    {
        if (diffCenter.norm() < params_.inner_radius()
                && (std::abs(diffCenter.z()) < params_.safety_height() / 2)
                && data.autopilotActive)
        {
            APLOG_DEBUG << "Restart Maneuver";
            startManeuver();
        }
    }
}

void
ManeuverPlanner::maneuverSetRequest(const std::string& maneuver)
{
    currentManeuverSet_ = maneuverSetMap_.find(maneuver);
    if (currentManeuverSet_ == maneuverSetMap_.end())
    {
        APLOG_ERROR << "Requested maneuver set " << maneuver << " not found.";
        return;
    }

    rollIterator_ = currentManeuverSet_->second.rollTargets.begin();
    pitchIterator_ = currentManeuverSet_->second.pitchTargets.begin();
    velocityIterator_ = currentManeuverSet_->second.velocityTargets.begin();

    overrideRestart_ = true;

    nextManeuverEvent_.cancel();

    startManeuver();
}

void
ManeuverPlanner::startManeuver()
{
    OverrideActivation activation;
    OverrideTarget target;

    if (rollIterator_ == currentManeuverSet_->second.rollTargets.end())
    {
        target.rollTarget = 0;
        activation.overrideRollTarget = false;
    }
    else
    {
        target.rollTarget = *rollIterator_;
        activation.overrideRollTarget = true;
    }

    if (pitchIterator_ == currentManeuverSet_->second.pitchTargets.end())
    {
        target.pitchTarget = 0;
        activation.overridePitchTarget = false;
    }
    else
    {
        target.pitchTarget = *pitchIterator_;
        activation.overridePitchTarget = true;
    }

    if (velocityIterator_ == currentManeuverSet_->second.velocityTargets.end())
    {
        target.velocityTarget = 0;
        activation.overrideVelocityTarget = false;
    }
    else
    {
        target.velocityTarget = *velocityIterator_;
        activation.overrideVelocityTarget = true;
    }
    activation.activate = true;

    overrideTargetPublisher_.publish(target);
    overrideActivationPublisher_.publish(activation);
    overrideActive_ = true;

    auto scheduler = scheduler_.get();
    if (!scheduler)
    {
        APLOG_ERROR << "Scheduler missing. Cannot schedule next maneuver.";
        return;
    }

    nextManeuverEvent_ = scheduler->schedule(std::bind(&ManeuverPlanner::nextManeuver, this),
                         currentManeuverSet_->second.maneuverDuration);
}
void
ManeuverPlanner::nextManeuver()
{
    const auto& maneuverSet = currentManeuverSet_->second;
    if (!maneuverSet.pitchTargets.empty())
    {
        ++pitchIterator_;
        if (pitchIterator_ != maneuverSet.pitchTargets.end())
        {
            startManeuver();
            return;
        }
        else
            pitchIterator_ = maneuverSet.pitchTargets.begin();
    }

    if (!maneuverSet.rollTargets.empty())
    {
        ++rollIterator_;
        if (rollIterator_ != maneuverSet.rollTargets.end())
        {
            startManeuver();
            return;
        }
        else
            rollIterator_ = maneuverSet.rollTargets.begin();
    }

    if (!maneuverSet.velocityTargets.empty())
    {
        ++velocityIterator_;
        if (velocityIterator_ != maneuverSet.velocityTargets.end())
        {
            startManeuver();
            return;
        }
        else
            velocityIterator_ = maneuverSet.velocityTargets.begin();
    }

    stopManeuver();

    APLOG_DEBUG << "Maneuvers finished.";
}

void
ManeuverPlanner::stopManeuver()
{
    APLOG_DEBUG << "Stop maneuver.";
    OverrideActivation activation;
    activation.activate = false;
    activation.overridePitchTarget = false;
    activation.overrideRollTarget = false;
    activation.overrideVelocityTarget = false;
    overrideRestart_ = false;
    overrideActive_ = false;
    overrideActivationPublisher_.publish(activation);
}

Rectangle
ManeuverPlanner::getSafetyRectangle() const
{
    return params_.safety_rectangle();
}

ControlOverride
ManeuverPlanner::getControlOverride() const
{
    return lastManualOverride_;
}

void
ManeuverPlanner::missionRequest(const std::string& mission)
{
    currentMission_ = missionMap_.find(mission);
    if (currentMission_ == missionMap_.end())
    {
        APLOG_ERROR << "Requested mission " << mission << " not found.";
        return;
    }

    publishMission();

}
