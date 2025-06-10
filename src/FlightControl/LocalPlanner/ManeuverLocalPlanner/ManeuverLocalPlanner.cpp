/**
 * @file ManeuverLocalPlanner.cpp
 * @date Aug 2, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief Description
 */

#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlanner.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/Core/OverrideHandler/OverrideHandler.h"
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>


void
ManeuverLocalPlanner::setTrajectory(const Trajectory& traj)
{
    Lock lock(trajectoryMutex_);
    trajectory_ = traj;
    if (trajectory_.periodicPart.empty())
    {
        trajectory_.periodicPart = safetyTrajectory_;
    }
    if (!trajectory_.aperiodicPart.empty())
        currentSection_ = trajectory_.aperiodicPart.begin();
    else
        currentSection_ = trajectory_.periodicPart.begin();
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
            if (!checkIsSet<IController, ISensingIO, IScheduler>())
            {
                CPSLOG_ERROR << "LinearLocalPlanner: Dependency missing";
                return true;
            }
            if (!isSet<DataHandling<Content, Target>>())
            {
                CPSLOG_DEBUG << "ManeuverLocalPlanner: DataHandling not set. Debugging disabled.";
            }
            if (!isSet<IPC>() && !isSet<DataPresentation>())
            {
                CPSLOG_DEBUG << "ManeuverLocalPlanner: IPC or DP not set. Will not subscribe on packets.";
            }
            auto safetyOrbit = std::make_shared<Orbit>(
                params.safetyOrbitCenter(), OrbitDirection::CCW, params.safetyOrbitRadius(), params.safetyVelocity());
            safetyTrajectory_ = {safetyOrbit};
            currentSection_ = safetyTrajectory_.begin();

            if (auto oh = get<OverrideHandler>())
            {
                oh->registerOverride("local_planner/velocity", velocityOverride_);
                oh->registerOverride("local_planner/position_x", positionXOverride_);
                oh->registerOverride("local_planner/position_y", positionYOverride_);
                oh->registerOverride("local_planner/position_z", positionZOverride_);
                oh->registerOverride("local_planner/direction_x", directionXOverride_);
                oh->registerOverride("local_planner/direction_y", directionYOverride_);
                oh->registerOverride("local_planner/curvature", curvatureOverride_);
                oh->registerOverride("local_planner/heading", headingOverride_);
                oh->registerOverride("local_planner/climbrate", climbrateOverride_);
                oh->registerOverride("controller_target/velocity", controllerTargetVelocityOverride_);
                oh->registerOverride("controller_target/climb_angle", controllerTargetClimbAngleOverride_);
                oh->registerOverride("controller_target/yaw_rate", controllerTargetYawRateOverride_);
            }
            else
                CPSLOG_DEBUG << "ManeuverLocalPlanner: OverrideHandler not set. Override disabled.";


            break;
        }
    case RunStage::NORMAL:
        {
            //Directly calculate local plan when sensor data comes in
            if (params.period() == 0)
            {
                CPSLOG_DEBUG << "Calculate control on sensor data trigger";
                auto sensing = get<ISensingIO>();
                sensing->subscribeOnSensorData([this](const SensorData& data)
                {
                    onSensorData(data);
                });
            }
            else
            {
                CPSLOG_DEBUG << "Calculate control with period " << params.period();
                auto scheduler = get<IScheduler>();
                scheduler->schedule([this] { update(); },
                                    Milliseconds(params.period()), Milliseconds(params.period()));
            }

            if (auto ipc = get<IPC>())
            {
                ipc->subscribeOnPackets("trajectory", [this](const Packet& packet)
                {
                    onTrajectoryPacket(packet);
                });
            }


            if (auto dh = get<DataHandling<Content, Target>>())
            {
                dh->addStatusFunction<ManeuverLocalPlannerStatus>(
                    [this] { return getStatus(); },
                    Content::MANEUVER_LOCAL_PLANNER_STATUS);
                dh->addConfig(this, Content::MANEUVER_LOCAL_PLANNER_PARAMS);
                dh->addTriggeredStatusFunction<Trajectory, DataRequest>([this](const DataRequest& request)
                              {
                                  return trajectoryRequest(request);
                              },
                              Content::TRAJECTORY, Content::REQUEST_DATA);
            }

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
ManeuverLocalPlanner::createLocalPlan(const SensorData& data)
{
    bool safety = false;

    Lock lock(trajectoryMutex_);
    auto currentSection = updatePathSection(data);
    lock.unlock();
    if (!currentSection)
    {
        CPSLOG_ERROR << "No current pathsection. Fly safety procedure.";
        safety = true;
    }

    if (!data.hasGPSFix)
    {
        CPSLOG_ERROR << "Lost GPS fix. LocalPlanner safety procedure.";
        safety = true;
    }

    if (safety)
    {
        controllerTarget_.velocity = params.safetyVelocity();
        controllerTarget_.yawRate = params.safetyOrbitRadius() / params.safetyVelocity();
        controllerTarget_.climbAngle = 0;
    }
    else
    {
        controllerTarget_ = calculateControllerTarget(data, currentSection);
    }

    //Do control overrides
    controllerTargetVelocityOverride_ = controllerTarget_.velocity;
    controllerTargetClimbAngleOverride_ = controllerTarget_.climbAngle;
    controllerTargetYawRateOverride_ = controllerTarget_.yawRate;

    controllerTarget_.velocity = controllerTargetVelocityOverride_;
    controllerTarget_.climbAngle = controllerTargetClimbAngleOverride_;
    controllerTarget_.yawRate = controllerTargetYawRateOverride_;


    status_.climbAngleTarget = controllerTarget_.climbAngle;
    status_.velocityTarget = controllerTarget_.velocity;
    status_.yawRateTarget = controllerTarget_.yawRate;
    status_.currentPathSection = currentSection;

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
ManeuverLocalPlanner::updatePathSection(const SensorData& data)
{
    auto currentSection = *currentSection_;

    if (!currentSection)
    {
        CPSLOG_ERROR << "Current Section is nullptr. Abort.";
        return nullptr;
    }
    currentSection->updateSensorData(data);

    if (currentSection->inTransition())
    {
        nextSection();
        currentSection = *currentSection_;
        if (!currentSection)
        {
            CPSLOG_ERROR << "Current Section is nullptr. Abort.";
            return nullptr;
        }
        currentSection->updateSensorData(data);
    }

    return currentSection;
}

void
ManeuverLocalPlanner::nextSection()
{
    ++currentSection_;
    if (currentSection_ == trajectory_.aperiodicPart.end() || currentSection_ == trajectory_.periodicPart.end())
    {
        currentSection_ = trajectory_.periodicPart.begin();
    }
}

ControllerTarget
ManeuverLocalPlanner::calculateControllerTarget(const SensorData& data,
                                                std::shared_ptr<IPathSection> section)
{
    ControllerTarget controllerTarget;

    velocityOverride_ = section->getVelocity();

    controllerTarget.velocity = velocityOverride_;
    Vector3 positionTarget = section->getPositionDeviation() + data.position;

    positionXOverride_ = positionTarget[0];
    positionYOverride_ = positionTarget[1];
    positionZOverride_ = positionTarget[2];

    Vector3 positionDeviation = Vector3(positionXOverride_(), positionYOverride_(), positionZOverride_()) - data.
        position;
    double distance = positionDeviation.norm();

    // Climb Rate
    double slope = section->getSlope();
    double climbRate = velocityOverride_ * slope * sqrt(1 / (1 + slope * slope))
        + params.kAltitude() * positionDeviation.z();

    climbrateOverride_ = std::clamp<FloatingType>(climbRate, -velocityOverride_, velocityOverride_);

    //Climb angle
    controllerTarget.climbAngle = asin(climbrateOverride_() / velocityOverride_());

    // Heading
    Vector3 direction = section->getDirection();

    Vector2 directionTarget = params.kConvergence() * positionDeviation.head(2)
        + direction.head(2).normalized();

    directionXOverride_ = directionTarget[0];
    directionYOverride_ = directionTarget[1];

    directionTarget = Vector2(directionXOverride_(), directionYOverride_());


    headingOverride_ = Angle<FloatingType>::fromRad(headingFromENU(directionTarget));

    if (distance > params.yawRateDistanceThreshold())
        curvatureOverride_ = 0;
    else
        curvatureOverride_ = section->getCurvature();


    double headingError = boundAngleRad(headingOverride_()() - data.attitude.z());

    // Yaw Rate

    controllerTarget.yawRate = velocityOverride_() * curvatureOverride_() + params.kYawRate() * headingError;

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
    }
    catch (ArchiveError& err)
    {
        CPSLOG_ERROR << "Invalid Trajectory packet: " << err.what();
        return;
    }

    if (auto dh = get<DataHandling<Content, Target>>())
    {
        dh->sendData(trajectory_, Content::TRAJECTORY);
    }
}

void
ManeuverLocalPlanner::onSensorData(const SensorData& sd)
{
    createLocalPlan(sd);
}

void
ManeuverLocalPlanner::update()
{
    auto sensing = get<ISensingIO>();

    if (!sensing)
    {
        CPSLOG_ERROR << "ManeuverLocalPlanner: sensing missing";
        return;
    }

    SensorData data = sensing->getSensorData();
    createLocalPlan(data);
}

Optional<Trajectory>
ManeuverLocalPlanner::trajectoryRequest(const DataRequest& request) const
{
    if (request == DataRequest::TRAJECTORY)
        return getTrajectory();
    return std::nullopt;
}
