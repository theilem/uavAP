/*
 * CustomPlanner.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#include <memory>
#include <mutex>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include "uavAP/MissionControl/MissionPlanner/CustomPlanner/CustomPlanner.h"
#include <cpsCore/Utilities/DataPresentation/detail/BasicSerialization.h>
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include <cpsCore/Utilities/IPC/IPC.h>

bool
CustomPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSet<IPC, IScheduler, IGlobalPlanner, DataHandling>())
            {
                CPSLOG_ERROR << "CustomPlanner: Missing deps.";
                return true;
            }

            //Creating default mission
            Waypoint centerWP;
            centerWP.location = Vector3(0, 0, params.defaultAltitude());
            Mission defaultMission;
            defaultMission.velocity = params.defaultVelocity();
            defaultMission.infinite = true;
            defaultMission.waypoints = {centerWP};

            auto defIt = params.missions().insert(std::make_pair("default_mission", defaultMission));

            auto it = params.missions().find(params.defaultMission());

            if (it == params.missions().end())
                currentMission_ = defIt.first;
            else
                currentMission_ = it;

            break;
        }
    case RunStage::NORMAL:
        {
            auto ipc = get<IPC>();

            sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
                                                                 std::bind(&CustomPlanner::onSensorData, this,
                                                                           std::placeholders::_1));

            if (!sensorDataSubscription_.connected())
            {
                CPSLOG_ERROR << "Sensor Data Missing.";
                return true;
            }

            if (auto dh = get<DataHandling>())
            {
                dh->subscribeOnData<std::string>(Content::SELECT_MISSION,
                                                 std::bind(&CustomPlanner::missionRequest, this,
                                                           std::placeholders::_1));
                dh->addTriggeredStatusFunction<std::vector<std::string>, DataRequest>(
                    [this](const DataRequest& req) -> Optional<std::vector<std::string>>
                    {
                        if (req == DataRequest::MISSION_LIST)
                        {
                            std::vector<std::string> ids;
                            for (const auto& it : params.missions())
                                ids.push_back(it.first);
                            return ids;
                        }
                        return std::nullopt;
                    }, Content::MISSION_LIST,
                    Content::REQUEST_DATA);
            }


            auto scheduler = get<IScheduler>();
            scheduler->schedule(std::bind(&CustomPlanner::publishMission, this), Milliseconds(100));
            break;
        }
    case RunStage::FINAL:
        {
            break;
        }
    default:
        break;
    }
    return false;
}

void
CustomPlanner::missionRequest(const std::string& mission)
{
    auto newMission = params.missions().find(mission);

    if (newMission == params.missions().end())
    {
        CPSLOG_ERROR << "Requested Mission " << mission << " Not Found.";
        return;
    }

    currentMission_ = newMission;

    publishMission();
}

void
CustomPlanner::publishMission()
{
    if (currentMission_ == params.missions().end())
    {
        CPSLOG_ERROR << "No Mission Selected. Cannot Publish.";
        return;
    }

    auto gp = get<IGlobalPlanner>();

    if (!gp)
    {
        CPSLOG_ERROR << "Cannot Set Mission. Global Planner Missing.";
        return;
    }

    CPSLOG_DEBUG << "Start Mission: " << currentMission_->first;

    Mission mission = currentMission_->second;

    if (params.useApproach())
    {
        std::unique_lock<std::mutex> lock(positionMutex_);
        Waypoint wp;
        wp.location = currentPosition_;
        wp.direction = currentDirection_;
    }

    if (mission.offset())
    {
        for (auto& wp : mission.waypoints())
        {
            wp.location() += *mission.offset();
        }
    }

    gp->setMission(mission);
}

void
CustomPlanner::onSensorData(const SensorData& data)
{
    LockGuard lockPosition(positionMutex_);
    currentPosition_ = data.position;
    currentDirection_ = directionFromAttitude(data.attitude);
}
