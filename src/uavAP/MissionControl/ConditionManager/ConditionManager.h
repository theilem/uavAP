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
 * ConditionManager.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITIONMANAGER_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITIONMANAGER_H_

#include <boost/signals2.hpp>

#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"

class IPC;
class IScheduler;

class ConditionManager: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "condition_manager";

	using OnSensorData = boost::signals2::signal<void (const SensorData&)>;
	using OnSteadyState = boost::signals2::signal<void (const Packet&)>;

	static std::shared_ptr<ConditionManager>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	boost::signals2::connection
	subscribeOnSensorData(const OnSensorData::slot_type& slot);

	boost::signals2::connection
	subscribeOnSteadyState(const OnSteadyState::slot_type& slot);

	void
	activateCondition(std::shared_ptr<ICondition> condition, const ICondition::ConditionTrigger& conditionTrigger);

	void
	deactivateCondition(std::shared_ptr<ICondition> condition);

	std::weak_ptr<IScheduler>
	getScheduler() const;

private:

	void
	onSensorData(const SensorData& sd);

	void
	onSteadyState(const Packet& packet);

	OnSensorData onSensorData_;
	OnSteadyState onSteadyState_;

	Subscription sensorDataSubscription_;
	Subscription steadyStateSubscription_;

	ObjectHandle<IPC> ipcHandle_;
	ObjectHandle<IScheduler> schedulerHandle_;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITIONMANAGER_H_ */
