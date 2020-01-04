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
 * SteadyStateAnalysis.h
 *
 *  Created on: Aug 14, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_STATEANALYSIS_STEADYSTATEANALYSIS_H_
#define UAVAP_FLIGHTANALYSIS_STATEANALYSIS_STEADYSTATEANALYSIS_H_

#include <uavAP/Core/LockTypes.h>
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Object/AggregatableObject.hpp"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"

class IPC;
class DataPresentation;

class SteadyStateAnalysis: public AggregatableObject<IPC, DataPresentation>, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "steady_state_analysis";

	static std::shared_ptr<SteadyStateAnalysis>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	SteadyStateMetrics
	getInspectingMetrics() const;

	void
	setInspectingMetrics(InspectingMetricsPair pair);

private:

	void
	onOverride(const Packet& packet);

	void
	onSensorData(const SensorData& data);

	void
	onPIDStati(const Packet& packet);

	void
	checkTolerance(const SensorData& data, const PIDStati& pidStati, const Override& override,
			Metrics& metrics);

	void
	checkSteadyState(const SensorData& data, const Override& override, Metrics& metrics);

	template<typename Enum>
	bool
	inTolerance(const double& value, const TimePoint& time, const std::map<Enum, double>& override,
			std::map<Enum, SteadyStateMetrics>& metrics, const Enum& member);

	template<typename Enum>
	bool
	inSteadyState(const TimePoint& time, const std::map<Enum, double>& override,
			std::map<Enum, SteadyStateMetrics>& metrics, const Enum& member);

	void
	resetMetrics(TimePoint time);

	Subscription sensorDataSubscription_;
	Subscription overrideSubscription_;
	Subscription pidStatiSubscription_;
	Publisher<Packet> steadyStatePublisher_;

	Override override_;
	PIDStati pidStati_;
	Metrics metrics_;
	SteadyStateMetrics* inspectingMetrics_;
	mutable SteadyStateMetrics lastInspectingMetrics_;

	Mutex overrideMutex_;
	Mutex pidStatiMutex_;
	Mutex metricsMutex_;
	mutable Mutex inspectingMetricsMutex_;

	TimePoint overrideTimeStamp_;

	double inToleranceDuration_;
	bool initialize_;
	bool reset_;
	bool newOverride_;
	bool emptyOverride_;
	bool lastEmptyOverride_;
};

template<typename Enum>
inline bool
SteadyStateAnalysis::inTolerance(const double& value, const TimePoint& time,
		const std::map<Enum, double>& override, std::map<Enum, SteadyStateMetrics>& metrics,
		const Enum& member)
{
	auto overridePair = findInMap(override, member);
	auto metricsPair = findInMap(metrics, member);

	if (overridePair && metricsPair)
	{
		auto target = overridePair->second;
		auto lastTarget = metricsPair->second.lastTarget;
		auto lastValue = metricsPair->second.lastValue;
		auto tolerance = metricsPair->second.tolerance;
		auto overshoot = metricsPair->second.overshoot;
		auto crossedTarget = metricsPair->second.crossedTarget;
		auto foundRiseTime = metricsPair->second.foundRiseTime;

		metricsPair->second.target = target;
		metricsPair->second.value = value;
		metricsPair->second.isReset = false;

		if (emptyOverride_ || lastEmptyOverride_)
		{
			metricsPair->second.foundLastTarget = false;
		}
		else if (!emptyOverride_ && !lastEmptyOverride_)
		{
			double targetDelta = std::fabs(target - lastTarget);

			if (targetDelta == 0)
			{
				metricsPair->second.foundLastTarget = false;
			}
			else
			{
				if (!crossedTarget)
				{
					metricsPair->second.crossedTarget = (lastValue <= target && value >= target)
							|| (lastValue >= target && value <= target);
				}

				if (metricsPair->second.crossedTarget)
				{
					metricsPair->second.overshoot = std::fmax(overshoot,
							(100 * std::fabs(target - value) / targetDelta));
				}

				metricsPair->second.foundLastTarget = true;
			}
		}

		if (std::fabs(target - value) <= tolerance)
		{
			metricsPair->second.inTolerance = true;

			if (!foundRiseTime)
			{
				metricsPair->second.riseTime = std::chrono::duration_cast<Milliseconds>(time - overrideTimeStamp_).count();
				metricsPair->second.foundRiseTime = true;
			}

			return true;
		}

		metricsPair->second.toleranceTimeStamp = time;
	}

	return false;
}

template<typename Enum>
inline bool
SteadyStateAnalysis::inSteadyState(const TimePoint& time, const std::map<Enum, double>& override,
		std::map<Enum, SteadyStateMetrics>& metrics, const Enum& member)
{
	auto overridePair = findInMap(override, member);
	auto metricsPair = findInMap(metrics, member);

	if (overridePair && metricsPair)
	{
		auto toleranceTimeStamp = metricsPair->second.toleranceTimeStamp;
		auto foundSettlingTime = metricsPair->second.foundSettlingTime;

		metricsPair->second.isReset = false;

		if (std::chrono::duration_cast<Milliseconds>(time - toleranceTimeStamp).count() > inToleranceDuration_)
		{
			metricsPair->second.inSteadyState = true;

			if (!foundSettlingTime)
			{
				metricsPair->second.settlingTime = std::chrono::duration_cast<Milliseconds>(time - overrideTimeStamp_).count()
						- inToleranceDuration_;

				metricsPair->second.foundSettlingTime = true;
			}

			return true;
		}

		return false;
	}

	return true;
}

#endif /* UAVAP_FLIGHTANALYSIS_STATEANALYSIS_STEADYSTATEANALYSIS_H_ */
