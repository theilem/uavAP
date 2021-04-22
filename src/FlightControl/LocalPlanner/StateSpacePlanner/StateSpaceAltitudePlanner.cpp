//
// Created by seedship on 4/19/21.
//

#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/Core/OverrideHandler/OverrideHandler.h"
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/StateSpaceAltitudePlanner.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/IPC/IPC.h>

StateSpaceAltitudePlanner::StateSpaceAltitudePlanner()
{
}

void
StateSpaceAltitudePlanner::setTrajectory(const Trajectory& traj)
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
StateSpaceAltitudePlanner::getTrajectory() const
{
	LockGuard lock(trajectoryMutex_);
	return trajectory_;
}

StateSpaceAltitudePlannerStatus
StateSpaceAltitudePlanner::getStatus() const
{
	return status_;
}

bool
StateSpaceAltitudePlanner::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<ISensingIO, IScheduler, IPC, DataPresentation, PitchStateSpaceController>())
			{
				CPSLOG_ERROR << "LinearLocalPlanner: Dependency missing";
				return true;
			}
			if (!isSet<DataHandling>())
			{
				CPSLOG_DEBUG << "ManeuverLocalPlanner: DataHandling not set. Debugging disabled.";
			}

			if (auto oh = get<OverrideHandler>())
			{
				oh->registerOverride("local_planner/velocity", velocity_);
				oh->registerOverride("local_planner/position_x", positionX_);
				oh->registerOverride("local_planner/position_y", positionY_);
				oh->registerOverride("local_planner/position_z", positionZ_);
				oh->registerOverride("local_planner/direction_x", directionX_);
				oh->registerOverride("local_planner/direction_y", directionY_);
				oh->registerOverride("local_planner/curvature", curvature_);
				oh->registerOverride("local_planner/heading", heading_);
				oh->registerOverride("local_planner/pitch", pitch_);
				oh->registerOverride("controller_target/velocity", controllerTargetVelocity_);
				oh->registerOverride("controller_target/pitch", controllerTargetPitch_);
				oh->registerOverride("controller_target/yaw_rate", controllerTargetYawRate_);
			}
			else
				CPSLOG_DEBUG << "ManeuverLocalPlanner: OverrideHandler not set. Override disabled.";

			if (auto pssc = get<PitchStateSpaceController>())
			{
				controllerURef_ = pssc->getUTrim();
				controllerThetaRef_ = pssc->getThetaTrim();
			}
			else
			{
				CPSLOG_ERROR << "Could not get PitchStateSpaceController! Quitting";
				return true;
			}

			break;
		}
		case RunStage::NORMAL:
		{

			//Directly calculate local plan when sensor data comes in
			if (params.period() == 0)
			{
				CPSLOG_DEBUG << "Calculate control on sensor data trigger";
				auto sensing = get<ISensingIO>();
				sensing->subscribeOnSensorData([this](const SensorData& sd)
											   {
												   createLocalPlan(sd);
											   });
			}
			else
			{
				CPSLOG_DEBUG << "Calculate control with period " << params.period();
				auto scheduler = get<IScheduler>();
				scheduler->schedule([this]()
									{ update(); }, Milliseconds(params.period()), Milliseconds(params.period()));
			}

			auto ipc = get<IPC>();

			ipc->subscribeOnPackets("trajectory", [this](const Packet& packet)
			{ onTrajectoryPacket(packet); });

			if (auto dh = get<DataHandling>())
			{
//				dh->addStatusFunction<StateSpaceAltitudePlannerStatus>([this]()
//																	   { return getStatus(); },
//																	   Content::STATE_SPACE_PLANNER_STATUS);
//				dh->addConfig(this, Content::MANEUVER_LOCAL_PLANNER_PARAMS);
				dh->addTriggeredStatusFunction<Trajectory, DataRequest>(
						[this](const DataRequest& request) -> Optional<Trajectory>
						{
							if (request == DataRequest::TRAJECTORY) {
								return getTrajectory();
							}
							return std::nullopt;
						}, Content::TRAJECTORY, Content::REQUEST_DATA);
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
StateSpaceAltitudePlanner::createLocalPlan(const SensorData& sd)
{
	Vector3 position = sd.position;
	double heading = sd.attitude.z();
	bool safety = false;

	Lock lock(trajectoryMutex_);
	auto currentSection = updatePathSection(position);
	if (!currentSection)
	{
		CPSLOG_ERROR << "No current pathsection. Fly safety procedure.";
		safety = true;
	}

	if (!sd.hasGPSFix)
	{
		CPSLOG_ERROR << "Lost GPS fix. LocalPlanner safety procedure.";
		safety = true;
	}

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

	// Assign values and get overrides
	controllerTarget_.velocity = (controllerTargetVelocity_ = controllerTarget_.velocity);
	controllerTarget_.climbAngle = (controllerTargetPitch_ = controllerTarget_.climbAngle);
	controllerTarget_.yawRate = (controllerTargetYawRate_ = controllerTarget_.yawRate);


	status_.pitchTarget = controllerTarget_.climbAngle;
	status_.velocityTarget = controllerTarget_.velocity;
	status_.yawRateTarget = controllerTarget_.yawRate;

	auto controller = get<PitchStateSpaceController>();
	if (!controller)
	{
		CPSLOG_ERROR << "Controller missing";
		return;
	}

	controller->setControllerTarget(controllerTarget_);
}

std::shared_ptr<IPathSection>
StateSpaceAltitudePlanner::updatePathSection(const Vector3& position)
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
StateSpaceAltitudePlanner::nextSection()
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
StateSpaceAltitudePlanner::calculateControllerTarget(const Vector3& position, double heading,
													 std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	controllerTarget.velocity = (velocity_ = section->getVelocity());
	Vector3 positionTarget = section->getPositionDeviation() + position;

	positionX_ = positionTarget[0];
	positionY_ = positionTarget[1];
	positionZ_ = positionTarget[2];

	Vector3 positionDeviation = Vector3(positionX_, positionY_, positionZ_) - position;
	double distance = positionDeviation.norm();

	// Pitch
	if (auto controller = get<PitchStateSpaceController>())
	{
		VectorN<7> state;// = {controller->getState(), positionDeviation.z()};
		auto val = controller->getState();
		// TODO fix with initializer lists when eigen 3.4 comes out
		state[0] = val[0];
		state[1] = val[1];
		state[2] = val[2];
		state[3] = val[3];
		state[4] = val[4];
		state[5] = val[5];
		state[6] = positionDeviation.z();
//		std::cout << position.z() << "," << positionDeviation.z() << "\n";
		auto out = params.k().dot(state);
		controllerTarget.climbAngle = (pitch_ = out + controllerThetaRef_);
		std::cout << "State[0]: (u) " << state[0] << "\n";
		std::cout << "State[1]: (w) " << state[1] << "\n";
		std::cout << "State[2]: (q) " << radToDeg(state[2]) << "\n";
		std::cout << "State[3]: (θ) " << radToDeg(state[3]) << "\n";
		std::cout << "State[4]: (Ep) " << radToDeg(state[4]) << "\n";
		std::cout << "State[5]: (Ev) " << state[5] << "\n";
		std::cout << "State[6]: (dh) " << state[6] << "\n";
		std::cout << "Pitch Target (degrees): " << radToDeg(controllerTarget.climbAngle) << "\n";
		std::cout << "U Target: " << controllerTarget.velocity << "\n";
	}
	else
	{
		CPSLOG_ERROR << "Missing Pitch State Space Controller, using safety pitch";
		controllerTarget.climbAngle = (pitch_ = controllerThetaRef_);
		controllerTarget.velocity = (velocity_ = section->getVelocity());
	}

	// Heading
	Vector3 direction = section->getDirection();

	Vector2 directionTarget = params.kConvergence() * positionDeviation.head(2)
							  + direction.head(2).normalized();

	directionX_ = directionTarget[0];
	directionY_ = directionTarget[1];

	directionTarget = Vector2(directionX_, directionY_());


	heading_ = Angle<FloatingType>::fromRad(headingFromENU(directionTarget));

	if (distance > params.yawRateDistanceThreshold())
		curvature_ = 0;
	else
		curvature_ = section->getCurvature();


	double headingError = boundAngleRad(heading_()() - heading);

	// Yaw Rate

	controllerTarget.yawRate = velocity_ * curvature_ + params.kYawRate() * headingError;

	return controllerTarget;
}

void
StateSpaceAltitudePlanner::onTrajectoryPacket(const Packet& packet)
{
	CPSLOG_DEBUG << "On Trajectory packet";
	auto dp = get<DataPresentation>();

	try
	{
		CPSLOG_DEBUG << "REACHED HERE\n";
		setTrajectory(dp->deserialize<Trajectory>(packet));
		CPSLOG_DEBUG << "REACHED HERE\n";
	} catch (ArchiveError& err)
	{
		CPSLOG_ERROR << "Invalid Trajectory packet: " << err.what();
		return;
	}
}

void
StateSpaceAltitudePlanner::update()
{
	if (auto sensing = get<ISensingIO>())
	{
		createLocalPlan(sensing->getSensorData());
	}
	else
	{
		CPSLOG_ERROR << "Missing Sensing";
	}
}

bool
StateSpaceAltitudePlanner::configure(const Configuration & c)
{
	std::cout << "Matrix K\n:" << params.k.value << "\n";
	auto retval = ConfigurableObject::configure(c);
	std::cout << "Matrix K\n:" << params.k.value << "\n";
	return retval;
}
