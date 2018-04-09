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
 * ManeuverCascade.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/ManeuverPIDController/detail/ManeuverCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h"
#include <map>

ManeuverCascade::ManeuverCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
                                 ControllerTarget* target, ControllerOutput* output, OverrideTarget* manTarget) :
    sensorData_(sensorData), controlEnv_(&sensorData->timestamp), hardRollConstraint_(30.0), hardPitchConstraint_(30.0)
{
    APLOG_TRACE << "Create ManeuverCascade";

    Control::PID::Parameters defaultParams;
    defaultParams.kp = 1;

    /* Yaw Rate Control */
    auto yawRateInput = controlEnv_.addInput(&sensorData->angularRate[2]);
    auto yawRateTarget = controlEnv_.addInput(&target->yawRate);
    auto yawRateTargetOverride = controlEnv_.addInput(&manTarget->yawRateTarget);

    switchYawRateTarget_ = std::make_shared<Control::ManualSwitch>(yawRateTargetOverride,
                           yawRateTarget);

    yawRatePID_ = controlEnv_.addPID(switchYawRateTarget_, yawRateInput, defaultParams);
    rollConstraint_ = controlEnv_.addConstraint(yawRatePID_, -hardRollConstraint_ * M_PI / 180.0,
                      hardRollConstraint_ * M_PI / 180.0);

    /* Roll Control */
    auto rollManeuverTarget = controlEnv_.addInput(&manTarget->rollTarget);
    auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]);
    auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[0]);

    switchRollTarget_ = std::make_shared<Control::ManualSwitch>(rollManeuverTarget,
                        rollConstraint_);

    rollPID_ = controlEnv_.addPID(switchRollTarget_, rollInput, rollRateInput, defaultParams);

    /* Roll Output */
    auto rollOverrideOutput = controlEnv_.addInput(&manTarget->rollOutput);

    switchRollOutput_ = std::make_shared<Control::ManualSwitch>(rollOverrideOutput,
                        rollPID_);

    auto rollOutConstraint = controlEnv_.addConstraint(switchRollOutput_, -1, 1);

    controlEnv_.addOutput(rollOutConstraint, &output->rollOutput);

    /* Climb Angle Control*/
    auto aoaInput = controlEnv_.addInput(&sensorData->angleOfAttack);
    auto pitchInput = controlEnv_.addInput(&sensorData->attitude[1]);

    auto climbAngle = controlEnv_.addDifference(pitchInput, aoaInput);

    auto climbAngleTarget = controlEnv_.addInput(&target->climbAngle);
    auto climbRateOverrideTarget = controlEnv_.addInput(&manTarget->climbRateTarget);

    switchClimbRateTarget_ = std::make_shared<Control::ManualSwitch>(climbRateOverrideTarget,
                             climbAngleTarget);

    climbAnglePID_ = controlEnv_.addPID(switchClimbRateTarget_, climbAngle,
                                        defaultParams);
    pitchConstraint_ = controlEnv_.addConstraint(climbAnglePID_,
                       -hardPitchConstraint_ * M_PI / 180.0, hardPitchConstraint_ * M_PI / 180.0);

    /* Pitch Control */
    auto pitchManeuverTarget = controlEnv_.addInput(&manTarget->pitchTarget);

    auto pitchRateInput = controlEnv_.addInput(&sensorData->angularRate[1]);

    switchPitchTarget_ = std::make_shared<Control::ManualSwitch>(pitchManeuverTarget,
                         pitchConstraint_);

    pitchPID_ = controlEnv_.addPID(switchPitchTarget_, pitchInput, pitchRateInput, defaultParams);

    /* Pitch Output */
    auto pitchOutOverride = controlEnv_.addInput(&manTarget->pitchOutput);

    switchPitchOutput_ = std::make_shared<Control::ManualSwitch>(pitchOutOverride,
                         pitchPID_);

    auto pitchOutConstraint = controlEnv_.addConstraint(switchPitchOutput_, -1, 1);

    controlEnv_.addOutput(pitchOutConstraint, &output->pitchOutput);

    /* Velocity Control */
    auto velocityManeuverTarget = controlEnv_.addInput(&manTarget->velocityTarget);

    auto velocityInput = controlEnv_.addInput(&sensorData->velocityGround);
    auto accelerationInput = controlEnv_.addInput(&sensorData->acceleration[0]);
    auto velocityTarget = controlEnv_.addInput(&target->velocity[0]);

    switchVelocityTarget_ = std::make_shared<Control::ManualSwitch>(velocityManeuverTarget,
                            velocityTarget);

    velocityPID_ = controlEnv_.addPID(switchVelocityTarget_, velocityInput, accelerationInput,
                                      defaultParams);


    /* Throttle Output */
    auto velocityOffset = controlEnv_.addConstant(1);
    auto velocityDifference = controlEnv_.addDifference(velocityPID_, velocityOffset);
    auto throttleOutOverride = controlEnv_.addInput(&manTarget->throttleOutput);

    switchThrottleOutput_ = std::make_shared<Control::ManualSwitch>(throttleOutOverride,
                            velocityDifference);

    auto velocityConstraint = controlEnv_.addConstraint(switchThrottleOutput_, -1, 1);
    controlEnv_.addOutput(velocityConstraint, &output->throttleOutput);


    /* Rudder Output */
    auto rudderBeta = controlEnv_.addInput(&beta_);
    auto rudderTarget = controlEnv_.addConstant(0);


    rudderPID_ = controlEnv_.addPID(rudderTarget, rudderBeta, defaultParams);

    auto invertedRudder = controlEnv_.addGain(rudderPID_, -1);

    auto rudderOverride = controlEnv_.addInput(&manTarget->yawOutput);

    switchYawOutput_ = std::make_shared<Control::ManualSwitch>(rudderOverride, invertedRudder);
    auto constraintYawOut = controlEnv_.addConstraint(switchYawOutput_, -1, 1);

    controlEnv_.addOutput(constraintYawOut, &output->yawOutput);

    /* Flap Output */
    auto flapConstant = controlEnv_.addConstant(-1); //Deactivated
    auto flapOverride = controlEnv_.addInput(&manTarget->flapOutput);

    switchFlapOutput_ = std::make_shared<Control::ManualSwitch>(flapOverride, flapConstant);
    auto constraintFlapOut = controlEnv_.addConstraint(switchFlapOutput_, -1, 1);

    controlEnv_.addOutput(constraintFlapOut, &output->flapOutput);

    //Disable Maneuver by default
    switchRollTarget_->switchTo(false);
    switchPitchTarget_->switchTo(false);
    switchVelocityTarget_->switchTo(false);
    switchClimbRateTarget_->switchTo(false);
    switchYawRateTarget_->switchTo(false);

    switchRollOutput_->switchTo(false);
    switchPitchOutput_->switchTo(false);
    switchYawOutput_->switchTo(false);
    switchThrottleOutput_->switchTo(false);
    switchFlapOutput_->switchTo(false);
}

bool
ManeuverCascade::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper pm(config);
    pm.add("hard_roll_constraint", hardRollConstraint_, false);
    pm.add("hard_pitch_constraint", hardPitchConstraint_, false);

    boost::property_tree::ptree pidConfig;
    pm.add("pids", pidConfig, false);

    rollConstraint_->setContraintValue(hardRollConstraint_ * M_PI / 180.0);
    pitchConstraint_->setContraintValue(hardPitchConstraint_ * M_PI / 180.0);

    Control::PID::Parameters params;
    for (auto it : pidConfig)
    {
        auto pid = AirplanePIDBimapRight.find(it.first);
        if (pid == AirplanePIDBimapRight.end())
        {
            APLOG_ERROR << it.first << " does not correspond to an airplane pid.";
            continue;
        }
        if (!params.configure(it.second))
        {
            APLOG_ERROR << it.first << " configuration not valid.";
            continue;
        }

        tunePID((int) pid->second, params);
    }
    return true;
}

bool
ManeuverCascade::tunePID(int pidIndicator, const Control::PID::Parameters& params)
{
    AirplanePIDs pid = static_cast<AirplanePIDs>(pidIndicator);

    switch (pid)
    {
    case AirplanePIDs::CLIMB_ANGLE:
        climbAnglePID_->setControlParameters(params);
        break;
    case AirplanePIDs::PITCH:
        pitchPID_->setControlParameters(params);
        break;
    case AirplanePIDs::YAW_RATE:
        yawRatePID_->setControlParameters(params);
        break;
    case AirplanePIDs::ROLL:
        rollPID_->setControlParameters(params);
        break;
    case AirplanePIDs::VELOCITY:
        velocityPID_->setControlParameters(params);
        break;
    case AirplanePIDs::RUDDER:
        rudderPID_->setControlParameters(params);
        break;
    default:
        APLOG_WARN << "Unknown pidIndicator. Ignore.";
        return false;
    }
    return true;
}

bool
ManeuverCascade::tuneRollBounds(double min, double max)
{
    if ((min < -hardRollConstraint_) || (min > 0.0))
    {
        APLOG_WARN << "Roll constraint min violates hard constraint.";
        return false;
    }
    if ((max > hardRollConstraint_) || (max < 0.0))
    {
        APLOG_WARN << "Roll constraint max violates hard constraint.";
        return false;
    }
    rollConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
    return true;
}

bool
ManeuverCascade::tunePitchBounds(double min, double max)
{
    if ((min < -hardPitchConstraint_) || (min > 0.0))
    {
        APLOG_WARN << "Pitch constraint min violates hard constraint.";
        return false;
    }
    if ((max > hardPitchConstraint_) || (max < 0.0))
    {
        APLOG_WARN << "Pitch constraint max violates hard constraint.";
        return false;
    }
    pitchConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
    return true;
}

std::map<int, PIDStatus>
ManeuverCascade::getPIDStatus()
{
    std::map<int, PIDStatus> status;
    status.insert(std::make_pair((int) AirplanePIDs::CLIMB_ANGLE, climbAnglePID_->getStatus()));
    status.insert(std::make_pair((int) AirplanePIDs::PITCH, pitchPID_->getStatus()));
    status.insert(std::make_pair((int) AirplanePIDs::ROLL, rollPID_->getStatus()));
    status.insert(std::make_pair((int) AirplanePIDs::VELOCITY, velocityPID_->getStatus()));
    status.insert(std::make_pair((int) AirplanePIDs::YAW_RATE, yawRatePID_->getStatus()));
    status.insert(std::make_pair((int) AirplanePIDs::RUDDER, rudderPID_->getStatus()));
    return status;
}

void
ManeuverCascade::evaluate()
{

    Vector3 velocityBody;
    velocityBody[0] = sensorData_->velocity.x();
    velocityBody[1] = sensorData_->velocity.y();
    velocityBody[2] = sensorData_->velocity.z();

    double yaw = -(sensorData_->attitude.z() - M_PI / 2);
    double roll = sensorData_->attitude.x();
    double pitch = - sensorData_->attitude.y();

    Eigen::Matrix3d m;
    m =  Eigen::AngleAxisd(-roll, Vector3::UnitX())
         * Eigen::AngleAxisd(-pitch, Vector3::UnitY())
         * Eigen::AngleAxisd(-yaw, Vector3::UnitZ());


    velocityBody = m * velocityBody;

    double bigV = velocityBody.norm();
    double smallV = velocityBody[1];

    beta_ = -asin(smallV / bigV);

    controlEnv_.evaluate();
}

void
ManeuverCascade::setManeuverOverride(const OverrideActivation& state)
{
    if (!state.activate)
    {
        switchRollTarget_->switchTo(false);
        switchPitchTarget_->switchTo(false);
        switchVelocityTarget_->switchTo(false);
        switchClimbRateTarget_->switchTo(false);
        switchYawRateTarget_->switchTo(false);

        switchRollOutput_->switchTo(false);
        switchPitchOutput_->switchTo(false);
        switchYawOutput_->switchTo(false);
        switchThrottleOutput_->switchTo(false);
        switchFlapOutput_->switchTo(false);
    }
    else
    {
        switchRollTarget_->switchTo(state.overrideRollTarget);
        switchPitchTarget_->switchTo(state.overridePitchTarget);
        switchVelocityTarget_->switchTo(state.overrideVelocityTarget);
        switchClimbRateTarget_->switchTo(state.overrideClimbRateTarget);
        switchYawRateTarget_->switchTo(state.overrideYawRateTarget);

        switchRollOutput_->switchTo(state.overrideRollOutput);
        switchPitchOutput_->switchTo(state.overridePitchOutput);
        switchYawOutput_->switchTo(state.overrideYawOutput);
        switchThrottleOutput_->switchTo(state.overrideThrottleOutput);
        switchFlapOutput_->switchTo(state.overrideFlapOutput);
    }
}
