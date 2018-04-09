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
/**
 *  @file         LinearLocalPlanner.cpp
 *  @author  Mirco Theile
 *  @date      27 June 2017
 *  @brief      UAV Autopilot Linear Local Planner Source File
 *
 *  Description
 */

#include <boost/thread/pthread/shared_mutex.hpp>
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/AirplaneLocalPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/HelicopterLocalPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/LockTypes.h"
#include <memory>

LinearLocalPlanner::LinearLocalPlanner() :
    airplane_(true),
    currentPathSectionIdx_(0)
{
}

std::shared_ptr<LinearLocalPlanner>
LinearLocalPlanner::create(const boost::property_tree::ptree& config)
{
    auto planner = std::make_shared<LinearLocalPlanner>();

    if (!planner->configure(config))
    {
        APLOG_ERROR << "LinearLocalPlanner: Configuration failed";
    }

    return planner;
}

bool
LinearLocalPlanner::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper propertyMapper(config);
    propertyMapper.add("airplane", airplane_, false);

    if (airplane_)
    {
        localPlannerImpl_ = std::make_shared<AirplaneLocalPlannerImpl>();
    }
    else
    {
        localPlannerImpl_ = std::make_shared<HelicopterLocalPlannerImpl>();
    }

    return localPlannerImpl_->configure(config);
}

bool
LinearLocalPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
    {
        if (!controller_.isSet())
        {
            APLOG_ERROR << "LinearLocalPlanner: Controller missing";

            return true;
        }
        if (!sensing_.isSet())
        {
            APLOG_ERROR << "LinearLocalPlanner: FlightControlData missing";

            return true;
        }
        if (!scheduler_.isSet())
        {
            APLOG_ERROR << "LinearLocalPlanner: Scheduler missing";

            return true;
        }
        if (!ipc_.isSet())
        {
            APLOG_ERROR << "LinearLocalPlanner: IPC missing";

            return true;
        }
        if (!dataPresentation_.isSet())
        {
            APLOG_ERROR << "LinearLocalPlanner: Data Presentation missing";

            return true;
        }

        break;
    }
    case RunStage::NORMAL:
    {
        auto sensing = sensing_.get();

        sensing->subscribeOnSensorData(boost::bind(&LinearLocalPlanner::createLocalPlan, this, _1));

//		if (localPlannerImpl_)
//		{
//			scheduler->schedule(std::bind(&LinearLocalPlanner::createLocalPlan, this),
//					Milliseconds(0), Milliseconds(10));
//		}

        auto ipc = ipc_.get();

        ipc->subscribeOnPacket("trajectory",
                               std::bind(&LinearLocalPlanner::onTrajectoryPacket, this, std::placeholders::_1));

        break;
    }
    case RunStage::FINAL:
    {
        break;
    }
    default:
    {
        break;
    }
    }
    return false;
}

void
LinearLocalPlanner::setTrajectory(const Trajectory& traj)
{
    LockGuard lock(trajectoryMutex_);

    trajectory_ = traj;
    currentSection_ = trajectory_.pathSections.begin();
    currentPathSectionIdx_ = 0;
    APLOG_DEBUG << "Trajectory set.";
}

ControllerTarget
LinearLocalPlanner::getControllerTarget()
{
    return controllerTarget_;
}

void
LinearLocalPlanner::nextSection()
{
    if (currentSection_ == trajectory_.pathSections.end())
    {
        return;
    }
    ++currentSection_;
    ++currentPathSectionIdx_;

    if (currentSection_ == trajectory_.pathSections.end() && trajectory_.infinite)
    {
        currentSection_ = trajectory_.pathSections.begin();
        currentPathSectionIdx_ = 0;
    }
}

void
LinearLocalPlanner::createLocalPlan(const SensorData& data)
{
//	auto sensing = sensing_.get();
//
//	if (!sensing)
//	{
//		APLOG_ERROR << "PIDController: FlightControlData missing";
//		return;
//	}
//
//	SharedLock sensorLock(sensing->mutex);
//	Vector3 position = sensing->sensorData.position;
//	double heading = sensing->sensorData.attitude.z();
//    bool hasFix = sensing->sensorData.hasGPSFix;
//    uint64_t seq = sensing->sensorData.sequenceNr;
//	sensorLock.unlock();

    Vector3 position = data.position;
    double heading = data.attitude.z();
    bool hasFix = data.hasGPSFix;
    uint32_t seq = data.sequenceNr;

    Lock lock(trajectoryMutex_);
    if (currentSection_ == trajectory_.pathSections.end())
    {
        APLOG_ERROR << "Trajectory at the end.";
        return;
    }

    auto currentSection = *currentSection_;
    if (!currentSection)
    {
        APLOG_ERROR << "Current Section is nullptr. Abort.";
        return;
    }
    currentSection->updatePosition(position);

    if (currentSection->inTransition())
    {
        nextSection();

        if (currentSection_ == trajectory_.pathSections.end())
        {
            APLOG_ERROR << "Trajectory at the end.";
            return;
        }

        currentSection = *currentSection_;
        if (!currentSection)
        {
            APLOG_ERROR << "Current Section is nullptr. Abort.";
            return;
        }
        currentSection->updatePosition(position);
    }
    lock.unlock();

    if (hasFix)
        controllerTarget_ = localPlannerImpl_->evaluate(position, heading, currentSection);
    else
    {
        APLOG_WARN << "Lost GPS fix. LocalPlanner safety procedure.";
        controllerTarget_.velocity[0] = currentSection->getVelocity();
        controllerTarget_.velocity[2] = 0;
        controllerTarget_.yawRate = 0;
    }
    controllerTarget_.sequenceNr = seq;

    auto controller = controller_.get();
    if (!controller)
    {
        APLOG_ERROR << "LinearLocalPlanner: Controller missing";

        return;
    }

    controller->setControllerTarget(controllerTarget_);
}

Trajectory
LinearLocalPlanner::getTrajectory() const
{
    return trajectory_;
}

std::shared_ptr<ILinearPlannerImpl>
LinearLocalPlanner::getImpl()
{
    return localPlannerImpl_;
}

LocalPlannerStatus
LinearLocalPlanner::getStatus()
{
    auto status = localPlannerImpl_->getStatus();
    if (!status.has_linear_status())
    {
        APLOG_ERROR << "Status from impl is wrong";
        return status;
    }

    status.mutable_linear_status()->set_current_path_section(currentPathSectionIdx_);
    status.mutable_linear_status()->mutable_velocity_target()->set_velocity_x(controllerTarget_.velocity.x());
    status.mutable_linear_status()->mutable_velocity_target()->set_velocity_y(controllerTarget_.velocity.y());
    status.mutable_linear_status()->mutable_velocity_target()->set_velocity_z(controllerTarget_.velocity.z());
    status.mutable_linear_status()->set_yaw_rate_target(controllerTarget_.yawRate);

    return status;
}

void
LinearLocalPlanner::notifyAggregationOnUpdate(Aggregator& agg)
{
    controller_.setFromAggregationIfNotSet(agg);
    sensing_.setFromAggregationIfNotSet(agg);
    scheduler_.setFromAggregationIfNotSet(agg);
    ipc_.setFromAggregationIfNotSet(agg);
    dataPresentation_.setFromAggregationIfNotSet(agg);
}

void
LinearLocalPlanner::onTrajectoryPacket(const Packet& packet)
{
    auto dp = dataPresentation_.get();

    if (!dp)
    {
        APLOG_ERROR << "Data Presentation missing. Cannot deserialize packet.";
        return;
    }

    Content content = Content::INVALID;
    auto any = dp->deserialize(packet, content);
    if (content != Content::TRAJECTORY)
    {
        APLOG_ERROR << "Invalid packet received. Content: " << (int) content;
        return;
    }

    setTrajectory(boost::any_cast<Trajectory>(any));

}
