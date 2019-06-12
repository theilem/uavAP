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

#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/AirplaneLocalPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Object/AggregatableObjectImpl.hpp"
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include <memory>

LinearLocalPlanner::LinearLocalPlanner() :
		inApproach_(false), airplane_(true), period_(0), currentPathSectionIdx_(0)
{
}

bool
LinearLocalPlanner::configure(const Configuration& config)
{
	PropertyMapper propertyMapper(config);
	propertyMapper.add<bool>("airplane", airplane_, false);
	propertyMapper.add<unsigned int>("period", period_, false);

	if (airplane_)
	{
		localPlannerImpl_ = std::make_shared<AirplaneLocalPlannerImpl>();
	}
	else
	{
		APLOG_ERROR << "Only airplane is defined";
//		localPlannerImpl_ = std::make_shared<HelicopterLocalPlannerImpl>();
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
		if (!isSet<IController>())
		{
			APLOG_ERROR << "LinearLocalPlanner: Controller missing";

			return true;
		}
		if (!isSet<ISensingActuationIO>())
		{
			APLOG_ERROR << "LinearLocalPlanner: FlightControlData missing";

			return true;
		}
		if (!isSet<IScheduler>())
		{
			APLOG_ERROR << "LinearLocalPlanner: Scheduler missing";

			return true;
		}
		if (!isSet<IPC>())
		{
			APLOG_ERROR << "LinearLocalPlanner: IPC missing";

			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{

		//Directly calculate local plan when sensor data comes in
		if (period_ == 0)
		{
			auto sensing = get<ISensingActuationIO>();
			sensing->subscribeOnSensorData(
					boost::bind(&LinearLocalPlanner::onSensorData, this, _1));
		}
		else
		{
			auto scheduler = get<IScheduler>();
			scheduler->schedule(std::bind(&LinearLocalPlanner::update, this), Milliseconds(period_),
					Milliseconds(period_));
		}

		auto ipc = get<IPC>();

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
	inApproach_ = trajectory_.approachSection != nullptr;
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
	if (inApproach_)
	{
		//We were approaching the first waypoint
		currentSection_ = trajectory_.pathSections.begin();
		inApproach_ = false;
		return;
	}

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

bool
LinearLocalPlanner::tune(const LocalPlannerParams& params)
{
	auto impl = getImpl();
	if (!impl)
	{
		APLOG_ERROR << "Impl missing. Cannot tune local planner.";
		return false;
	}
	return impl->tuneParams(params);
}

void
LinearLocalPlanner::createLocalPlan(const Vector3& position, double heading, bool hasGPSFix,
		uint32_t seqNum)
{

	Lock lock(trajectoryMutex_);
	std::shared_ptr<IPathSection> currentSection;

	if (!inApproach_)
	{
		if (currentSection_ == trajectory_.pathSections.end())
		{
			APLOG_ERROR << "Trajectory at the end.";
			return;
		}
		currentSection = *currentSection_;
	}
	else
	{
		currentSection = trajectory_.approachSection;
	}

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

	if (hasGPSFix)
		controllerTarget_ = localPlannerImpl_->evaluate(position, heading, currentSection);
	else
	{
		APLOG_WARN << "Lost GPS fix. LocalPlanner safety procedure.";
		controllerTarget_.velocity = currentSection->getVelocity();
		controllerTarget_.yawRate = 0;
	}
	controllerTarget_.sequenceNr = seqNum;

	auto controller = get<IController>();
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
LinearLocalPlanner::getStatus() const
{
	auto status = localPlannerImpl_->getStatus();
	if (!status.has_linear_status())
	{
		APLOG_ERROR << "Status from impl is wrong";
		return status;
	}

	status.mutable_linear_status()->set_current_path_section(currentPathSectionIdx_);
	status.mutable_linear_status()->mutable_velocity_target()->set_velocity_x(
			controllerTarget_.velocity);
	status.mutable_linear_status()->mutable_velocity_target()->set_velocity_y(0);
	status.mutable_linear_status()->mutable_velocity_target()->set_velocity_z(0);
	status.mutable_linear_status()->set_yaw_rate_target(controllerTarget_.yawRate);
	status.mutable_linear_status()->set_is_in_approach(inApproach_);

	return status;
}

void
LinearLocalPlanner::onTrajectoryPacket(const Packet& packet)
{
	try
	{
		setTrajectory(dp::deserialize<Trajectory>(packet));
	} catch (ArchiveError& err)
	{
		APLOG_ERROR << "Invalid Trajectory packet: " << err.what();
		return;
	}
}

void
LinearLocalPlanner::onSensorData(const SensorData& sd)
{
	//TODO Lock?
	Vector3 position = sd.position;
	double heading = sd.attitude.z();
	bool hasFix = sd.hasGPSFix;
	uint32_t seq = sd.sequenceNr;

	createLocalPlan(position, heading, hasFix, seq);
}

void
LinearLocalPlanner::update()
{
	auto sensing = get<ISensingActuationIO>();

	if (!sensing)
	{
		APLOG_ERROR << "PIDController: FlightControlData missing";
		return;
	}

	SensorData data = sensing->getSensorData();
	createLocalPlan(data.position, data.attitude.z(), data.hasGPSFix, data.sequenceNr);
}
