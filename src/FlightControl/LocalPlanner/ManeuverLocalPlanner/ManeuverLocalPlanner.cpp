/**
 * @file ManeuverLocalPlanner.cpp
 * @date Aug 2, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief Description
 */

#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include "uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlanner.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>


void
ManeuverLocalPlanner::setTrajectory(const Trajectory& traj)
{
	Lock lock(trajectoryMutex_);
	trajectory_ = traj;
	currentSection_ = trajectory_.pathSections.begin();
	lock.unlock();

	Lock lockStatus(statusMutex_);
	status_.currentPathSection = 0;
	status_.isInApproach = trajectory_.approachSection != nullptr;
	lockStatus.unlock();

	CPSLOG_DEBUG << "Trajectory set.";
}

Trajectory
ManeuverLocalPlanner::getTrajectory() const
{
	LockGuard lock(trajectoryMutex_);
	return trajectory_;
}

ManeuverLocalPlannerStatus
ManeuverLocalPlanner::getStatus() const
{
	return status_;
}

bool
ManeuverLocalPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IController, ISensingActuationIO, IScheduler, IPC, DataPresentation>())
		{
			CPSLOG_ERROR << "LinearLocalPlanner: Dependency missing";
			return true;
		}
		if (!isSet<DataHandling>())
		{
			CPSLOG_DEBUG << "ManeuverLocalPlanner: DataHandling not set. Debugging disabled.";
		}

		break;
	}
	case RunStage::NORMAL:
	{

		//Directly calculate local plan when sensor data comes in
		if (params.period() == 0)
		{
			CPSLOG_DEBUG << "Calculate control on sensor data trigger";
			auto sensing = get<ISensingActuationIO>();
			sensing->subscribeOnSensorData(
					boost::bind(&ManeuverLocalPlanner::onSensorData, this, _1));
		}
		else
		{
			CPSLOG_DEBUG << "Calculate control with period " << params.period();
			auto scheduler = get<IScheduler>();
			scheduler->schedule(std::bind(&ManeuverLocalPlanner::update, this),
					Milliseconds(params.period()), Milliseconds(params.period()));
		}

		auto ipc = get<IPC>();

		ipc->subscribeOnPackets("trajectory",
				std::bind(&ManeuverLocalPlanner::onTrajectoryPacket, this, std::placeholders::_1));

		ipc->subscribeOnPackets("override",
				std::bind(&ManeuverLocalPlanner::onOverridePacket, this, std::placeholders::_1));

		if (auto dh = get<DataHandling>())
		{
			dh->addStatusFunction<ManeuverLocalPlannerStatus>(
					std::bind(&ManeuverLocalPlanner::getStatus, this),
					Content::MANEUVER_LOCAL_PLANNER_STATUS);
			dh->addConfig(this, Content::MANEUVER_LOCAL_PLANNER_PARAMS);
			dh->addTriggeredStatusFunction<Trajectory, DataRequest>(
					std::bind(&ManeuverLocalPlanner::trajectoryRequest, this,
							std::placeholders::_1), Content::TRAJECTORY, Content::REQUEST_DATA);
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
ManeuverLocalPlanner::createLocalPlan(const Vector3& position, double heading, bool hasGPSFix)
{
	bool safety = false;

	Lock lock(trajectoryMutex_);
	auto currentSection = updatePathSection(position);
	if (!currentSection)
	{
		CPSLOG_ERROR << "No current pathsection. Fly safety procedure.";
		safety = true;
	}

	if (!hasGPSFix)
	{
		CPSLOG_ERROR << "Lost GPS fix. LocalPlanner safety procedure.";
		safety = true;
	}

	Lock plannerLock(overrideMutex_);

	if (safety)
	{
		controllerTarget_.velocity = params.safetyVelocity();
		controllerTarget_.yawRate = params.safetyYawRate();
		controllerTarget_.climbAngle = 0;
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

//	controllerTarget_.sequenceNr = seqNum;
	status_.climbAngleTarget = controllerTarget_.climbAngle;
	status_.velocityTarget = controllerTarget_.velocity;
	status_.yawRateTarget = controllerTarget_.yawRate;

	auto controller = get<IController>();
	if (!controller)
	{
		CPSLOG_ERROR << "LinearLocalPlanner: Controller missing";
		return;
	}

	if (params.doOverrideVelocity())
	{
		controllerTarget_.velocity = params.overrideVelocity();
	}

	controller->setControllerTarget(controllerTarget_);
}

std::shared_ptr<IPathSection>
ManeuverLocalPlanner::updatePathSection(const Vector3& position)
{
	std::shared_ptr<IPathSection> currentSection;

	if (!status_.isInApproach)
	{
		if (currentSection_ == trajectory_.pathSections.end())
		{
			CPSLOG_ERROR << "Trajectory at the end.";
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
		CPSLOG_ERROR << "Current Section is nullptr. Abort.";
		return nullptr;
	}
	currentSection->updatePosition(position);

	if (currentSection->inTransition())
	{
		nextSection();

		if (currentSection_ == trajectory_.pathSections.end())
		{
			CPSLOG_ERROR << "Trajectory at the end.";
			return nullptr;
		}

		currentSection = *currentSection_;
		if (!currentSection)
		{
			CPSLOG_ERROR << "Current Section is nullptr. Abort.";
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
	if (status_.isInApproach)
	{
		//We were approaching the first waypoint
		currentSection_ = trajectory_.pathSections.begin();
		status_.isInApproach = false;
		return;
	}

	if (currentSection_ == trajectory_.pathSections.end())
	{
		return;
	}
	++currentSection_;
	++status_.currentPathSection;

	if (currentSection_ == trajectory_.pathSections.end() && trajectory_.infinite)
	{
		currentSection_ = trajectory_.pathSections.begin();
		status_.currentPathSection = 0;
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
			+ params.kAltitude() * positionDeviation.z();

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

	Vector2 directionTarget = params.kConvergence() * positionDeviation.head(2)
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

	controllerTarget.yawRate = vel * curvature + params.kYawRate() * headingError;

	return controllerTarget;
}

void
ManeuverLocalPlanner::onTrajectoryPacket(const Packet& packet)
{
	CPSLOG_DEBUG << "On Trajectory packet";
	auto dp = get<DataPresentation>();

	try
	{
		setTrajectory(dp->deserialize<Trajectory>(packet));
	} catch (ArchiveError& err)
	{
		CPSLOG_ERROR << "Invalid Trajectory packet: " << err.what();
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
//	uint32_t seq = sd.sequenceNr;

	createLocalPlan(position, heading, hasFix);
}

void
ManeuverLocalPlanner::onOverridePacket(const Packet& packet)
{
	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "DataPresentation missing";
		return;
	}

	auto override = dp->deserialize<Override>(packet);

	std::unique_lock<std::mutex> plannerLock(overrideMutex_);
	plannerOverrides_ = override.localPlanner;
	targetOverrides_ = override.controllerTarget;
}

void
ManeuverLocalPlanner::update()
{
	auto sensing = get<ISensingActuationIO>();

	if (!sensing)
	{
		CPSLOG_ERROR << "ManeuverLocalPlanner: sensing missing";
		return;
	}

	SensorData data = sensing->getSensorData();
	createLocalPlan(data.position, data.attitude.z(), data.hasGPSFix);
}

Optional<Trajectory>
ManeuverLocalPlanner::trajectoryRequest(const DataRequest& request)
{
	if (request == DataRequest::TRAJECTORY)
		return getTrajectory();
	return std::nullopt;
}
