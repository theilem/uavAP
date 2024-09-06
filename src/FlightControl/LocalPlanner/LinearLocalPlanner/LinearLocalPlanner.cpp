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

#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "cpsCore/Utilities/Scheduler/IScheduler.h"
#include <memory>

LinearLocalPlanner::LinearLocalPlanner() :
		headingTarget_(0), inApproach_(false), currentPathSectionIdx_(0)
{
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
			CPSLOG_ERROR << "LinearLocalPlanner: Controller missing";

			return true;
		}
		if (!isSet<ISensingIO>())
		{
			CPSLOG_ERROR << "LinearLocalPlanner: FlightControlData missing";

			return true;
		}
		if (!isSet<IScheduler>())
		{
			CPSLOG_DEBUG << "LinearLocalPlanner: Scheduler missing. Can only react to SensingIO";
		}

		Trajectory traj;
		traj.pathSections.push_back(
				std::make_shared<Orbit>(Vector3(0, 0, 0), Vector3(0, 0, 1), 50, 50));
		setTrajectory(traj);

		break;
	}
	case RunStage::NORMAL:
	{

		//Directly calculate local plan when sensor data comes in
		if (params.period() == 0 || !isSet<IScheduler>())
		{
			auto sensing = get<ISensingIO>();
			sensing->subscribeOnSensorData(std::bind(&LinearLocalPlanner::onSensorData, this, std::placeholders::_1));
		}
		else
		{
			auto scheduler = get<IScheduler>();
			scheduler->schedule(std::bind(&LinearLocalPlanner::update, this),
					Milliseconds(params.period()), Milliseconds(params.period()));
		}

		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
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
	CPSLOG_TRACE << "Trajectory set.";
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

void
LinearLocalPlanner::createLocalPlan(const SensorData& data)
{
	Lock lock(trajectoryMutex_);
	std::shared_ptr<IPathSection> currentSection;

	if (!inApproach_)
	{
		if (currentSection_ == trajectory_.pathSections.end())
		{
			CPSLOG_ERROR << "Trajectory at the end.";
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
		CPSLOG_ERROR << "Current Section is nullptr. Abort.";
		return;
	}
	currentSection->updateSensorData(data);

	if (currentSection->inTransition())
	{
		nextSection();

		if (currentSection_ == trajectory_.pathSections.end())
		{
			CPSLOG_ERROR << "Trajectory at the end.";
			return;
		}

		currentSection = *currentSection_;
		if (!currentSection)
		{
			CPSLOG_ERROR << "Current Section is nullptr. Abort.";
			return;
		}
		currentSection->updateSensorData(data);
	}
	lock.unlock();

	if (data.hasGPSFix)
		controllerTarget_ = evaluate(data.position, data.attitude.z(), currentSection);
	else
	{
		CPSLOG_WARN << "Lost GPS fix. LocalPlanner safety procedure.";
		controllerTarget_.velocity = currentSection->getVelocity();
		controllerTarget_.yawRate = 0;
	}
//	controllerTarget_.sequenceNr = seqNum;

	auto controller = get<IController>();
	if (!controller)
	{
		CPSLOG_ERROR << "LinearLocalPlanner: Controller missing";

		return;
	}

	controller->setControllerTarget(controllerTarget_);
}

Trajectory
LinearLocalPlanner::getTrajectory() const
{
	return trajectory_;
}

void
LinearLocalPlanner::onSensorData(const SensorData& sd)
{
	createLocalPlan(sd);
}

void
LinearLocalPlanner::update()
{
	auto sensing = get<ISensingIO>();

	if (!sensing)
	{
		CPSLOG_ERROR << "PIDController: FlightControlData missing";
		return;
	}

	SensorData data = sensing->getSensorData();
	createLocalPlan(data);
}

ControllerTarget
LinearLocalPlanner::evaluate(const Vector3& position, FloatingType heading,
		std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	FloatingType vel = section->getVelocity();

	controllerTarget.velocity = vel;
	auto positionDeviation = section->getPositionDeviation();

	// Climb Rate
	FloatingType climbRate = controllerTarget.velocity * section->getSlope()
			+ params.kAltitude() * positionDeviation.z();

	climbRate = climbRate > vel ? vel : climbRate < -vel ? -vel : climbRate;

	//Climb angle
	controllerTarget.climbAngle = asin(climbRate / vel);

	// Heading
	Vector2 directionTarget_ = params.kHeading() * positionDeviation.head(2)
			+ section->getDirection().head(2).normalized();
	headingTarget_ = headingFromENU(directionTarget_);

	FloatingType headingError = boundAngleRad(headingTarget_ - heading);

	// Yaw Rate
	controllerTarget.yawRate = vel * section->getCurvature() + params.kYawrate() * headingError;
	return controllerTarget;
}
