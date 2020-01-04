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
 * MissionControlDataHandling.h
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_
#define UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Time.h"

class DataPresentation;
class IPC;
class IScheduler;
class IGlobalPlanner;
class ManeuverPlanner;
class IMissionPlanner;
class LocalFrameManager;
class ConditionManager;
class WindAnalysis;
class Geofencing;

class MissionControlDataHandling: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "mc_data_handling";

	MissionControlDataHandling();

	static std::shared_ptr<MissionControlDataHandling>
	create(const Configuration& configuration);

	bool
	configure(const Configuration& configuration);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:
	void
	collectAndSend();

	void
	receiveAndDistribute(const Packet& packet);

	void
	collectAndSendMission(std::shared_ptr<DataPresentation> dp);

	void
	collectAndSendSafetyBounds(std::shared_ptr<DataPresentation> dp);

	void
	collectAndSendLocalFrame(std::shared_ptr<DataPresentation> dp);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IGlobalPlanner> globalPlanner_;
	ObjectHandle<ManeuverPlanner> maneuverPlanner_;
	ObjectHandle<IMissionPlanner> missionPlanner_;
	ObjectHandle<DataPresentation> dataPresentation_;
	ObjectHandle<LocalFrameManager> localFrameManager_;
	ObjectHandle<ConditionManager> conditionManager_;
	ObjectHandle<WindAnalysis> windAnalysis_;
	ObjectHandle<Geofencing> geofencing_;

	Subscription missionControlSubscription_;
	Publisher<Packet> publisher_;
	Publisher<Packet> overridePublisher_;

	unsigned int lastOverrideSeqNr_;
	Duration period_;
};

#endif /* UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_ */
