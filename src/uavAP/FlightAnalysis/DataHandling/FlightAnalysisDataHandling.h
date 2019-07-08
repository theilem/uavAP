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
 * FlightAnalysisDataHandling.h
 *
 *  Created on: Jul 23, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_DATAHANDLING_FLIGHTANALYSISDATAHANDLING_H_
#define UAVAP_FLIGHTANALYSIS_DATAHANDLING_FLIGHTANALYSISDATAHANDLING_H_

#include "uavAP/Core/DataPresentation/ContentMapping.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"
#include "uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h"

class FlightAnalysisDataHandling: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "fa_data_handling";

	FlightAnalysisDataHandling();

	static std::shared_ptr<FlightAnalysisDataHandling>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

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
	collectAndSendInspectingMetrics(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	setInspectingMetrics(InspectingMetricsPair pair);

	ObjectHandle<IPC> ipcHandle_;
	ObjectHandle<IScheduler> schedulerHandle_;
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentationHandle_;
	ObjectHandle<SteadyStateAnalysis> steadyStateAnalysisHandle_;

	Subscription subscription_;
	Publisher publisher_;

	Duration period_;
};

#endif /* UAVAP_FLIGHTANALYSIS_DATAHANDLING_FLIGHTANALYSISDATAHANDLING_H_ */
