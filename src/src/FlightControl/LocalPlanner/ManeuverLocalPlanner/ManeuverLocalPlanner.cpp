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
 * ManeuverLocalPlanner.cpp
 *
 *  Created on: Aug 2, 2018
 *      Author: mircot
 */
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/PropertyMapper/PropertyMapperProto.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/FlightControl/Controller/IController.h>
#include <uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h>
#include "uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlanner.h"
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>

ManeuverLocalPlanner::ManeuverLocalPlanner() :
		period_(0)
{
}

bool
ManeuverLocalPlanner::configure(const boost::property_tree::ptree& config)
{
	PropertyMapperProto pm(config);
	pm.add<unsigned int>("period", period_, false);
	pm.addProto("", params_, true);
	pm.add<double>("safety_velocity", safetyTarget_.velocity, true);
	pm.add<double>("safety_yaw_rate", safetyTarget_.yawRate, false);
	return pm.map();
}

void
ManeuverLocalPlanner::setTrajectory(const Trajectory& traj)
{
	Lock lock(trajectoryMutex_);
	trajectory_ = traj;
	currentSection_ = trajectory_.pathSections.begin();
	lock.unlock();

	Lock lockStatus(statusMutex_);
	status_.set_current_path_section(0);
	status_.set_is_in_approach(trajectory_.approachSection != nullptr);
	lockStatus.unlock();

	APLOG_DEBUG << "Trajectory set.";
}

Trajectory
ManeuverLocalPlanner::getTrajectory() const
{
	LockGuard lock(trajectoryMutex_);
	return trajectory_;
}

LocalPlannerStatus
ManeuverLocalPlanner::getStatus() const
{
	LocalPlannerStatus status;

	Lock lock(statusMutex_);
	status.mutable_maneuver_status()->CopyFrom(status_);
	lock.unlock();

	return status;
}

bool
ManeuverLocalPlanner::tune(const LocalPlannerParams& params)
{
	if (!params.has_maneuver_params())
	{
		APLOG_ERROR
				<< "ManeuverLocalPlanner::tune: Tuning params do not contain Maneuver params. Ignore.";
		return false;
	}

	LockGuard lock(paramsMutex_);
	params_.CopyFrom(params.maneuver_params());
	return true;
}

bool
ManeuverLocalPlanner::run(RunStage stage)
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
		if (!dataHandling_.isSet())
		{
			APLOG_DEBUG << "ManeuverLocalPlanner: DataHandling not set. Debugging disabled.";
		}

		break;
	}
	case RunStage::NORMAL:
	{

		//Directly calculate local plan when sensor data comes in
		if (period_ == 0)
		{
			APLOG_DEBUG << "Calculate control on sensor data trigger";
			auto sensing = sensing_.get();
			sensing->subscribeOnSensorData(
					boost::bind(&ManeuverLocalPlanner::onSensorData, this, _1));
		}
		else
		{
			APLOG_DEBUG << "Calculate control with period " << period_;
			auto scheduler = scheduler_.get();
			scheduler->schedule(std::bind(&ManeuverLocalPlanner::update, this),
					Milliseconds(period_), Milliseconds(period_));
		}

		auto ipc = ipc_.get();

		ipc->subscribeOnPacket("trajectory",
				std::bind(&ManeuverLocalPlanner::onTrajectoryPacket, this, std::placeholders::_1));

		ipc->subscribeOnPacket("override",
				std::bind(&ManeuverLocalPlanner::onOverridePacket, this, std::placeholders::_1));

		if (auto dh = dataHandling_.get())
		{
			dh->addStatusFunction<LocalPlannerStatus>(
					std::bind(&ManeuverLocalPlanner::getStatus, this));
			dh->subscribeOnCommand<LocalPlannerParams>(Content::TUNE_LOCAL_PLANNER,
					std::bind(&ManeuverLocalPlanner::tune, this, std::placeholders::_1));
			dh->addTriggeredStatusFunction<Trajectory, DataRequest>(
					std::bind(&ManeuverLocalPlanner::trajectoryRequest, this,
							std::placeholders::_1), Content::REQUEST_DATA);
		}

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
ManeuverLocalPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
	controller_.setFromAggregationIfNotSet(agg);
	sensing_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
	dataHandling_.setFromAggregationIfNotSet(agg);
}

void
ManeuverLocalPlanner::createLocalPlan(const Vector3& position, double heading, bool hasGPSFix,
		uint32_t seqNum)
{
	bool safety = false;

	Lock lock(trajectoryMutex_);
	auto currentSection = updatePathSection(position);
	if (!currentSection)
	{
		APLOG_ERROR << "No current pathsection. Fly safety procedure.";
		safety = true;
	}

	if (!hasGPSFix)
	{
		APLOG_ERROR << "Lost GPS fix. LocalPlanner safety procedure.";
		safety = true;
	}

	std::unique_lock<std::mutex> plannerLock(overrideMutex_);

	if (safety)
	{
		controllerTarget_ = safetyTarget_;
	}
	else
	{
		controllerTarget_ = calculateControllerTarget(position, heading, currentSection);
	}

	//Do control overrides
	if (auto it = findInMap(targetOverrides_, ControllerTargets::VELOCITY))
		controllerTarget_.velocity = it->second;
	if (auto it = findInMap(targetOverrides_, ControllerTargets::CLIMB_ANGLE))
		controllerTarget_.climbAngle = it->second;
	if (auto it = findInMap(targetOverrides_, ControllerTargets::YAW_RATE))
		controllerTarget_.yawRate = it->second;

	plannerLock.unlock();

	controllerTarget_.sequenceNr = seqNum;
	status_.set_climb_angle_target(controllerTarget_.climbAngle);
	status_.set_velocity_target(controllerTarget_.velocity);
	status_.set_yaw_rate_target(controllerTarget_.yawRate);

	auto controller = controller_.get();
	if (!controller)
	{
		APLOG_ERROR << "LinearLocalPlanner: Controller missing";
		return;
	}

	controller->setControllerTarget(controllerTarget_);
}

std::shared_ptr<IPathSection>
ManeuverLocalPlanner::updatePathSection(const Vector3& position)
{
	std::shared_ptr<IPathSection> currentSection;

	if (!status_.is_in_approach())
	{
		if (currentSection_ == trajectory_.pathSections.end())
		{
			APLOG_ERROR << "Trajectory at the end.";
			return nullptr;
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
		return nullptr;
	}
	currentSection->updatePosition(position);

	if (currentSection->inTransition())
	{
		nextSection();

		if (currentSection_ == trajectory_.pathSections.end())
		{
			APLOG_ERROR << "Trajectory at the end.";
			return nullptr;
		}

		currentSection = *currentSection_;
		if (!currentSection)
		{
			APLOG_ERROR << "Current Section is nullptr. Abort.";
			return nullptr;
		}
		currentSection->updatePosition(position);
	}

	return currentSection;

}

void
ManeuverLocalPlanner::nextSection()
{
	//Status mutex is locked already
	if (status_.is_in_approach())
	{
		//We were approaching the first waypoint
		currentSection_ = trajectory_.pathSections.begin();
		status_.set_is_in_approach(false);
		return;
	}

	if (currentSection_ == trajectory_.pathSections.end())
	{
		return;
	}
	++currentSection_;
	status_.set_current_path_section(status_.current_path_section() + 1);

	if (currentSection_ == trajectory_.pathSections.end() && trajectory_.infinite)
	{
		currentSection_ = trajectory_.pathSections.begin();
		status_.set_current_path_section(0);
	}
}

ControllerTarget
ManeuverLocalPlanner::calculateControllerTarget(const Vector3& position, double heading,
		std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	double vel = section->getVelocity();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::VELOCITY))
		vel = it->second;

	controllerTarget.velocity = vel;
	auto positionDeviation = section->getPositionDeviation();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_X))
		positionDeviation[0] = it->second - position[0];
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_Y))
		positionDeviation[1] = it->second - position[1];
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_Z))
		positionDeviation[2] = it->second - position[2];

	// Climb Rate
	double slope = section->getSlope();
	double climbRate = vel * slope * sqrt(1 / (1 + slope * slope))
			+ params_.k_altitude() * positionDeviation.z();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::SLOPE))
	{
		//Override climbrate, shall not approach altitude
		double slope = it->second;
		climbRate = vel * slope * sqrt(1 / (1 + slope * slope));
	}

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::CLIMB_RATE))
		climbRate = it->second;

	climbRate = climbRate > vel ? vel : climbRate < -vel ? -vel : climbRate;

	//Climb angle
	controllerTarget.climbAngle = asin(climbRate / vel);

	// Heading
	Vector3 direction = section->getDirection();
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_X))
		direction[0] = it->second;
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_Y))
		direction[1] = it->second;
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_Z))
		direction[2] = it->second;

	Vector2 directionTarget = params_.k_convergence() * positionDeviation.head(2)
			+ direction.head(2).normalized();
	double headingTarget = headingFromENU(directionTarget);

	double curvature = section->getCurvature();
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::HEADING))
	{
		headingTarget = it->second;
		curvature = 0;
	}

	double headingError = boundAngleRad(headingTarget - heading);

	// Yaw Rate

	controllerTarget.yawRate = vel * curvature + params_.k_yaw_rate() * headingError;

	return controllerTarget;
}

void
ManeuverLocalPlanner::onTrajectoryPacket(const Packet& packet)
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
ManeuverLocalPlanner::onSensorData(const SensorData& sd)
{
	//TODO Lock?
	Vector3 position = sd.position;
	double heading = sd.attitude.z();
	bool hasFix = sd.hasGPSFix;
	uint32_t seq = sd.sequenceNr;

	createLocalPlan(position, heading, hasFix, seq);
}

void
ManeuverLocalPlanner::onOverridePacket(const Packet& packet)
{
	auto override = dp::deserialize<Override>(packet);

	std::unique_lock<std::mutex> plannerLock(overrideMutex_);
	plannerOverrides_ = override.localPlanner;
	targetOverrides_ = override.controllerTarget;
}

void
ManeuverLocalPlanner::update()
{
	auto sensing = sensing_.get();

	if (!sensing)
	{
		APLOG_ERROR << "ManeuverLocalPlanner: sensing missing";
		return;
	}

	SensorData data = sensing->getSensorData();
	createLocalPlan(data.position, data.attitude.z(), data.hasGPSFix, data.sequenceNr);
}

boost::optional<Trajectory>
ManeuverLocalPlanner::trajectoryRequest(const DataRequest& request)
{
	APLOG_DEBUG << "Called trajectoryRequest with " << (int)request;
	if (request == DataRequest::TRAJECTORY)
		return getTrajectory();
	return boost::none;
}
