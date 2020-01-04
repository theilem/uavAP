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
 * SteadyStateAnalysis.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: simonyu
 */
#include "uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h"
#include "uavAP/Core/Object/AggregatableObjectImpl.hpp"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/DataPresentation/DataPresentation.h"

std::shared_ptr<SteadyStateAnalysis>
SteadyStateAnalysis::create(const Configuration& config)
{
	auto steadyStateAnalysis = std::make_shared<SteadyStateAnalysis>();

	if (!steadyStateAnalysis->configure(config))
	{
		APLOG_ERROR << "SteadyStateAnalysis: Failed to Load Config.";
	}

	return steadyStateAnalysis;
}

bool
SteadyStateAnalysis::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Configuration toleranceTree;

	pm.add("tolerance", toleranceTree, false);
	pm.add<double>("in_tolerance_duration", inToleranceDuration_, false);

	metrics_.configure(toleranceTree);
	inspectingMetrics_ = nullptr;
	initialize_ = false;
	reset_ = true;
	newOverride_ = false;

	return pm.map();
}

bool
SteadyStateAnalysis::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, DataPresentation>())
		{
			APLOG_ERROR << "SteadyStateAnalysis: Missing dependencies.";
			return true;
		}

		if (auto ipc = get<IPC>())
		{
			steadyStatePublisher_ = ipc->publishPackets("steady_state");
		}
		else
		{
			APLOG_ERROR << "SteadyStateAnalysis: IPC Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		if (!ipc)
		{
			APLOG_ERROR << "SteadyStateAnalysis: IPC Missing.";
			return true;
		}

		sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&SteadyStateAnalysis::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "SteadyStateAnalysis: Sensor Data Missing.";
			return true;
		}

		overrideSubscription_ = ipc->subscribeOnPackets("active_override",
				std::bind(&SteadyStateAnalysis::onOverride, this, std::placeholders::_1));

		if (!overrideSubscription_.connected())
		{
			APLOG_ERROR << "SteadyStateAnalysis: Override Missing.";
			return true;
		}

		pidStatiSubscription_ = ipc->subscribeOnPackets("pid_stati",
				std::bind(&SteadyStateAnalysis::onPIDStati, this, std::placeholders::_1));

		if (!pidStatiSubscription_.connected())
		{
			APLOG_ERROR << "SteadyStateAnalysis: PID Stati Missing.";
			return true;
		}

		break;
	}
	default:
		break;
	}

	return false;
}

SteadyStateMetrics
SteadyStateAnalysis::getInspectingMetrics() const
{
	LockGuard lg(inspectingMetricsMutex_);

	if (inspectingMetrics_)
	{
		lastInspectingMetrics_ = *inspectingMetrics_;
	}

	return lastInspectingMetrics_;
}

void
SteadyStateAnalysis::setInspectingMetrics(InspectingMetricsPair pair)
{
	MetricsGroup inspectingGroup = static_cast<MetricsGroup>(pair.first);

	Lock lock(metricsMutex_);
	switch (inspectingGroup)
	{
	case MetricsGroup::LOCAL_PLANNER:
	{
		LocalPlannerTargets inspectingMember = static_cast<LocalPlannerTargets>(pair.second);

		if (auto metricsPair = findInMap(metrics_.localPlanner, inspectingMember))
		{
			Lock lock(inspectingMetricsMutex_);
			inspectingMetrics_ = &(metricsPair->second);
			lock.unlock();
		}
		else
		{
			APLOG_WARN << "SteadyStateAnalysis: Invalid Inspecting Member.";
		}

		break;
	}
	case MetricsGroup::CONTROLLER_TARGETS:
	{
		ControllerTargets inspectingMember = static_cast<ControllerTargets>(pair.second);

		if (auto metricsPair = findInMap(metrics_.controllerTarget, inspectingMember))
		{
			Lock lock(inspectingMetricsMutex_);
			inspectingMetrics_ = &(metricsPair->second);
			lock.unlock();
		}
		else
		{
			APLOG_WARN << "SteadyStateAnalysis: Invalid Inspecting Member.";
		}

		break;
	}
	case MetricsGroup::PIDS:
	{
		PIDs inspectingMember = static_cast<PIDs>(pair.second);

		if (auto metricsPair = findInMap(metrics_.pid, inspectingMember))
		{
			Lock lock(inspectingMetricsMutex_);
			inspectingMetrics_ = &(metricsPair->second);
			lock.unlock();
		}
		else
		{
			APLOG_WARN << "SteadyStateAnalysis: Invalid Inspecting Member.";
		}

		break;
	}
	case MetricsGroup::INVALID:
	{
		APLOG_WARN << "SteadyStateAnalysis: Invalid Inspecting Group.";
		break;
	}
	default:
	{
		APLOG_WARN << "SteadyStateAnalysis: Unknown Inspecting Group.";
		break;
	}
	}
	lock.unlock();
}

void
SteadyStateAnalysis::onOverride(const Packet& packet)
{
	if (auto dp = get<DataPresentation>())
	{
		Lock lock(overrideMutex_);
		override_ = dp->deserialize<Override>(packet);
		newOverride_ = true;
		lock.unlock();
	}
}

void
SteadyStateAnalysis::onPIDStati(const Packet& packet)
{
	if (auto dp = get<DataPresentation>())
	{
		Lock lock(pidStatiMutex_);
		pidStati_ = dp->deserialize<PIDStati>(packet);
		lock.unlock();
	}
}

void
SteadyStateAnalysis::onSensorData(const SensorData& data)
{
	if (!initialize_)
	{
		Lock metricsLock(metricsMutex_);
		metrics_.setToleranceTimeStamp(data.timestamp);
		metricsLock.unlock();

		initialize_ = true;
	}

	Lock overrideLock(overrideMutex_);

	if (newOverride_ || override_.isEmpty())
	{
		if (newOverride_)
		{
			lastEmptyOverride_ = emptyOverride_;
			emptyOverride_ = override_.isEmpty();
		}

		if (reset_)
		{
			resetMetrics(data.timestamp);
		}

		overrideTimeStamp_ = data.timestamp;
		metrics_.setToleranceTimeStamp(data.timestamp);
		newOverride_ = false;
	}

	Lock pidStatiLock(pidStatiMutex_);
	Lock metricsLock(metricsMutex_);

	checkTolerance(data, pidStati_, override_, metrics_);
	checkSteadyState(data, override_, metrics_);

	metricsLock.unlock();
	pidStatiLock.unlock();
	overrideLock.unlock();

	reset_ = true;
}

void
SteadyStateAnalysis::checkTolerance(const SensorData& data, const PIDStati& pidStati,
		const Override& override, Metrics& metrics)
{
	inTolerance<LocalPlannerTargets>(data.position.x(), data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_X);
	inTolerance<LocalPlannerTargets>(data.position.y(), data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_Y);
	inTolerance<LocalPlannerTargets>(data.position.z(), data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_Z);
	inTolerance<LocalPlannerTargets>(data.airSpeed, data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::VELOCITY);
	inTolerance<LocalPlannerTargets>(data.attitude.z(), data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::HEADING);
	inTolerance<LocalPlannerTargets>(data.velocity.z(), data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::CLIMB_RATE);

	inTolerance<ControllerTargets>(data.airSpeed, data.timestamp, override.controllerTarget,
			metrics.controllerTarget, ControllerTargets::VELOCITY);
	inTolerance<ControllerTargets>(data.angularRate.z(), data.timestamp, override.controllerTarget,
			metrics.controllerTarget, ControllerTargets::YAW_RATE);

	auto climbAnglePair = findInMap(pidStati, PIDs::CLIMB_ANGLE);

	if (climbAnglePair)
	{
		inTolerance<ControllerTargets>(climbAnglePair->second.value, data.timestamp,
				override.controllerTarget, metrics.controllerTarget,
				ControllerTargets::CLIMB_ANGLE);
	}

	auto velocityPair = findInMap(pidStati, PIDs::VELOCITY);
	auto velocityXPair = findInMap(pidStati, PIDs::VELOCITY_X);
	auto velocityYPair = findInMap(pidStati, PIDs::VELOCITY_Y);
	auto rudderPair = findInMap(pidStati, PIDs::RUDDER);
	auto climbRatePair = findInMap(pidStati, PIDs::CLIMB_RATE);
	auto pitchPair = findInMap(pidStati, PIDs::PITCH);
	auto pitchRatePair = findInMap(pidStati, PIDs::PITCH_RATE);
	auto yawRatePair = findInMap(pidStati, PIDs::YAW_RATE);
	auto rollPair = findInMap(pidStati, PIDs::ROLL);
	auto rollRatePair = findInMap(pidStati, PIDs::ROLL_RATE);

	if (velocityPair)
	{
		inTolerance<PIDs>(velocityPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::VELOCITY);
	}

	if (velocityXPair)
	{
		inTolerance<PIDs>(velocityXPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::VELOCITY_X);
	}

	if (velocityYPair)
	{
		inTolerance<PIDs>(velocityYPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::VELOCITY_Y);
	}

	if (rudderPair)
	{
		inTolerance<PIDs>(rudderPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::RUDDER);
	}

	if (climbAnglePair)
	{
		inTolerance<PIDs>(climbAnglePair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::CLIMB_ANGLE);
	}

	if (climbRatePair)
	{
		inTolerance<PIDs>(climbRatePair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::CLIMB_RATE);
	}

	if (pitchPair)
	{
		inTolerance<PIDs>(pitchPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::PITCH);
	}

	if (pitchRatePair)
	{
		inTolerance<PIDs>(pitchRatePair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::PITCH_RATE);
	}

	if (yawRatePair)
	{
		inTolerance<PIDs>(yawRatePair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::YAW_RATE);
	}

	if (rollPair)
	{
		inTolerance<PIDs>(rollPair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::ROLL);
	}

	if (rollRatePair)
	{
		inTolerance<PIDs>(rollRatePair->second.value, data.timestamp, override.pid, metrics.pid,
				PIDs::ROLL_RATE);
	}
}

void
SteadyStateAnalysis::checkSteadyState(const SensorData& data, const Override& override,
		Metrics& metrics)
{
	bool steadyState = true;

	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_X);
	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_Y);
	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::POSITION_Z);
	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::VELOCITY);
	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::HEADING);
	steadyState &= inSteadyState<LocalPlannerTargets>(data.timestamp, override.localPlanner,
			metrics.localPlanner, LocalPlannerTargets::CLIMB_RATE);

	steadyState &= inSteadyState<ControllerTargets>(data.timestamp, override.controllerTarget,
			metrics.controllerTarget, ControllerTargets::VELOCITY);
	steadyState &= inSteadyState<ControllerTargets>(data.timestamp, override.controllerTarget,
			metrics.controllerTarget, ControllerTargets::YAW_RATE);
	steadyState &= inSteadyState<ControllerTargets>(data.timestamp, override.controllerTarget,
			metrics.controllerTarget, ControllerTargets::CLIMB_ANGLE);

	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::VELOCITY);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::VELOCITY_X);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::VELOCITY_Y);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::RUDDER);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid,
			PIDs::CLIMB_ANGLE);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::CLIMB_RATE);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::PITCH);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::PITCH_RATE);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::YAW_RATE);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::ROLL);
	steadyState &= inSteadyState<PIDs>(data.timestamp, override.pid, metrics.pid, PIDs::ROLL_RATE);

	if (auto dp = get<DataPresentation>())
	{
		steadyStatePublisher_.publish(dp->serialize(steadyState));
	}
}

void
SteadyStateAnalysis::resetMetrics(TimePoint time)
{

	Lock lock(metricsMutex_);
	metrics_.reset();
	lock.unlock();

	reset_ = false;
}
