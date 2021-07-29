//
// Created by seedship on 7/25/21.
//

#include "uavAP/Core/OverrideHandler/OverrideHandler.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include <uavAP/Core/Orientation/NED.h>
#include <uavAP/Core/Orientation/ConversionUtils.h>
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/RSLQRPlanner/RSLQRPlanner.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/RSLQRController/RSLQRController.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

RSLQRPlanner::RSLQRPlanner(): controlEnv_(&sensorData_.timestamp)
{

}

void
RSLQRPlanner::setTrajectory(const Trajectory& traj)
{
	Lock lock(trajectoryMutex_);
	trajectory_ = traj;
	currentSection_ = trajectory_.pathSections.begin();
	lock.unlock();
}

Trajectory
RSLQRPlanner::getTrajectory() const
{
	LockGuard lock(trajectoryMutex_);
	return trajectory_;
}

bool
RSLQRPlanner::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			populateControlEnv();
			if (!checkIsSet<ISensingIO, IScheduler, IPC, DataPresentation, RSLQRController>())
			{
				CPSLOG_ERROR << "Missing Dependencies.";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "DataHandling not set. Debugging disabled.";
			}
			if (auto oh = get<OverrideHandler>())
			{
				oh->registerOverride("local_planner/position_x", positionTargetE_);
				oh->registerOverride("local_planner/position_y", positionTargetN_);
				oh->registerOverride("local_planner/position_z", positionTargetU_);
				oh->registerOverride("local_planner/direction_x", directionE_);
				oh->registerOverride("local_planner/direction_y", directionN_);
				oh->registerOverride("local_planner/heading_target", headingTarget_);
				oh->registerOverride("controller_target/velocity", controllerTargetVelocity_);
				oh->registerOverride("controller_target/pitch", controllerTargetPitch_);
				oh->registerOverride("controller_target/roll", controllerTargetRoll_);
			}
			else
				CPSLOG_DEBUG << "OverrideHandler not set. Overrides disabled.";
			break;
		}
		case RunStage::NORMAL:
		{
			if (auto dh = get<DataHandling>())
			{
				dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
						[this](){return getStatus();}, Content::PID_STATUS);
			}
			auto sensing = get<ISensingIO>();
			sensing->subscribeOnSensorData([this](const SensorData& sd)
										   {
											   onSensorData(sd);
										   });
			auto ipc = get<IPC>();
			ipc->subscribeOnPackets("trajectory", [this](const Packet& packet)
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
			});

			if (auto dh = get<DataHandling>())
			{
				dh->addStatusFunction<ManeuverLocalPlannerStatus>([this]()
																  { return status_; },
																  Content::MANEUVER_LOCAL_PLANNER_STATUS);
				dh->addTriggeredStatusFunction<Trajectory, DataRequest>(
						[this](const DataRequest& request) -> Optional<Trajectory>
						{
							if (request == DataRequest::TRAJECTORY) {
								return getTrajectory();
							}
							return std::nullopt;
						}, Content::TRAJECTORY, Content::REQUEST_DATA);
			}
		}
		default:
		{
			break;
		}
	}
	return false;
}

void
RSLQRPlanner::populateControlEnv()
{
	auto dPitch = controlEnv_.addInput(&stateOut_[0]);
	auto dRoll = controlEnv_.addInput(&stateOut_[1]);

	auto pitchInt = controlEnv_.addIntegrator(dPitch, params.pitchParams());
	auto rollInt = controlEnv_.addIntegrator(dRoll, params.rollParams());

	auto pitchOut = controlEnv_.addOutput(pitchInt, &outputPitch_);
	auto rollOut = controlEnv_.addOutput(rollInt, &outputRoll_);
}

// Copied from ManeuverLocalPlanner
std::shared_ptr<IPathSection>
RSLQRPlanner::updatePathSection(const Vector3& position_enu)
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
	currentSection->updatePosition(position_enu);

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
		currentSection->updatePosition(position_enu);
	}

	return currentSection;
}


// Copied from ManeuverLocalPlanner
void
RSLQRPlanner::nextSection()
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

// Copied from ManeuverLocalPlanner
void
RSLQRPlanner::createLocalPlan(const Vector3& position_enu, FloatingType heading_ned, bool hasGPSFix, FloatingType vz, FloatingType psi_dot)
{
	bool safety = false;

	Lock l(trajectoryMutex_);
	auto currentSection = updatePathSection(position_enu);
	l.unlock();
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


	if (safety)
	{
		controllerTarget_.velocity = params.safetyVelocity();
		controllerTarget_.yawRate = 0;
		controllerTarget_.climbAngle = 0;
	}
	else
	{
		controllerTarget_ = calculateControllerTarget(position_enu, heading_ned, currentSection, vz, psi_dot);
	}

	if (auto controller = get<RSLQRController>())
		controller->setControllerTarget(controllerTarget_);
	else
		CPSLOG_ERROR << "Missing RSLQRController!";
}

ControllerTarget
RSLQRPlanner::calculateControllerTarget(const Vector3& position_enu, double heading_ned,
										std::shared_ptr<IPathSection> section, FloatingType vz, FloatingType psi_dot)
{
	ControllerTarget controllerTarget;

	controllerTarget.velocity = (controllerTargetVelocity_ = section->getVelocity());
	Vector3 positionTarget = section->getPositionDeviation() + position_enu;

	Lock l(targetMutex_);

	positionTargetE_ = positionTarget[0];
	positionTargetN_ = positionTarget[1];
	positionTargetU_ = positionTarget[2];

	Vector3 positionDeviation = Vector3(positionTargetE_, positionTargetN_, positionTargetU_) - position_enu;

	// Heading
	Vector3 direction = section->getDirection();

	Vector2 directionTarget = params.kConvergence() * positionDeviation.head(2)
							  + direction.head(2).normalized();

	directionE_ = directionTarget[0];
	directionN_ = directionTarget[1];

	directionTarget = Vector2(directionE_, directionN_());

	//ENU Heading
	auto enuHeadingTarget = headingFromENU(directionTarget);
	headingTarget_ = Angle<FloatingType>::fromRad(boundAngleRad(-enuHeadingTarget + degToRad(90)));

	l.unlock();

	// Pitch
	if (auto controller = get<RSLQRController>())
	{
		VectorN<16> state;// = {controller->getState(), positionDeviation.z()};
		auto controllerState = controller->getState();
		// TODO fix with initializer lists when eigen 3.4 comes out
		state[0] = -positionDeviation.z(); // error defined as current - target
		state[1] = boundAngleRad(heading_ned - headingTarget_()());
//		std::cout << "Heading:" << heading_ned << "\n";
//		std::cout << "Heading Target:" << headingTarget_()() << "\n";
//		std::cout << "Heading Error:" << state[1] << "\n";
		state[2] = controllerState[0];
		state[3] = controllerState[1];
		state[4] = controllerState[2];
		state[5] = controllerState[3];
		state[6] = controllerState[4];
		state[7] = controllerState[5];
		state[8] = controllerState[6];
		state[9] = controllerState[7];
		state[10] = controllerState[8];
		state[11] = controllerState[9];
		state[12] = controllerState[10];
		state[13] = controllerState[11];
		state[14] = -vz;
		state[15] = psi_dot;
		stateOut_ = -params.k() * state;
		controlEnv_.evaluate();
		controllerTarget.climbAngle = (controllerTargetPitch_ = outputPitch_);
		controllerTarget.yawRate = (controllerTargetRoll_ = outputRoll_);
	}
	else
	{
		CPSLOG_ERROR << "Missing Pitch State Space Controller, using safety";
		controllerTarget.climbAngle = (controllerTargetPitch_ = 0);
		controllerTarget.yawRate = (controllerTargetRoll_ = 0);
	}

	return controllerTarget;
}

void
RSLQRPlanner::onSensorData(const SensorData& sd)
{
	Lock l(sensorDataMutex_);
	sensorData_ = sd;
	Vector3 pos_enu = sd.position;
	NED::convert(sensorData_, Frame::BODY);
	FloatingType heading_ned = sensorData_.attitude[2];
	bool hasGPSFix = sd.hasGPSFix;
	FramedVector3 inertialVel = sensorData_.velocity;
	directionalConversion(inertialVel, sensorData_.attitude, Frame::INERTIAL, Orientation::NED);
	FramedVector3 inertialAttRate = sensorData_.angularRate;
	angularConversion(inertialAttRate, sensorData_.attitude, Frame::INERTIAL, Orientation::NED);
	l.unlock();

	createLocalPlan(pos_enu, heading_ned, hasGPSFix, inertialVel.z(), inertialAttRate.z());
}

std::map<PIDs, PIDStatus>
RSLQRPlanner::getStatus() const
{
	if (auto controller = get<RSLQRController>()) {
		std::map<PIDs, PIDStatus> ans = controller->getStatus();
		Lock sdm(sensorDataMutex_);
		Lock t(targetMutex_);
		ans[PIDs::ALTITUDE] = PIDStatus(positionTargetU_, -sensorData_.position.z());
		ans[PIDs::HEADING] = PIDStatus(headingTarget_().degrees(), radToDeg(sensorData_.attitude[2]));
		return ans;
	}
	return decltype(getStatus())();
}
