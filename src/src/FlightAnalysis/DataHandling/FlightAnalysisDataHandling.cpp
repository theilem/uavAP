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
 * FlightAnalysisDataHandling.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: simonyu
 */

#include "uavAP/FlightAnalysis/DataHandling/FlightAnalysisDataHandling.h"

FlightAnalysisDataHandling::FlightAnalysisDataHandling() :
		period_(Milliseconds(100))
{
}

std::shared_ptr<FlightAnalysisDataHandling>
FlightAnalysisDataHandling::create(const boost::property_tree::ptree& config)
{
	auto dataHandling = std::make_shared<FlightAnalysisDataHandling>();

	if (!dataHandling->configure(config))
	{
		APLOG_ERROR << "DataHandling: Failed to Load Config.";
	}

	return dataHandling;
}

bool
FlightAnalysisDataHandling::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add("period", period_, false);

	return pm.map();
}

bool
FlightAnalysisDataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!schedulerHandle_.isSet())
		{
			APLOG_ERROR << "DataHandling: Scheduler Missing.";
			return true;
		}

		if (!ipcHandle_.isSet())
		{
			APLOG_ERROR << "DataHandling: IPC Missing.";
			return true;
		}

		if (!dataPresentationHandle_.isSet())
		{
			APLOG_ERROR << "DataHandling: Data Presentation Missing.";
			return true;
		}

		auto ipc = ipcHandle_.get();

		publisher_ = ipc->publishPackets("data_fa_com");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipcHandle_.get();

		subscription_ = ipc->subscribeOnPacket("data_com_fa",
				boost::bind(&FlightAnalysisDataHandling::receiveAndDistribute, this, _1));

		if (!subscription_.connected())
		{
			APLOG_WARN << "DataHandling: Communication Missing. Ignoring.";

			return true;
		}

		auto scheduler = schedulerHandle_.get();
		scheduler->schedule(std::bind(&FlightAnalysisDataHandling::collectAndSend, this),
				Milliseconds(0), period_);

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
FlightAnalysisDataHandling::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipcHandle_.setFromAggregationIfNotSet(agg);
	schedulerHandle_.setFromAggregationIfNotSet(agg);
	dataPresentationHandle_.setFromAggregationIfNotSet(agg);
	steadyStateAnalysisHandle_.setFromAggregationIfNotSet(agg);
}

void
FlightAnalysisDataHandling::collectAndSend()
{
	auto dp = dataPresentationHandle_.get();

	if (!dp)
	{
		APLOG_ERROR << "Data Presentation Missing. Cannot Collect or Send.";
		return;
	}

	collectAndSendInspectingMetrics(dp);
}

void
FlightAnalysisDataHandling::receiveAndDistribute(const Packet& packet)
{
	auto dp = dataPresentationHandle_.get();

	if (!dp)
	{
		APLOG_ERROR << "Data Presentation Missing. Cannot Receive or Distribute.";
		return;
	}

	Content content = Content::INVALID;
	auto any = dp->deserialize(packet, content);

	try
	{
		switch (content)
		{
		case Content::SELECT_INSPECTING_METRICS:
		{
			setInspectingMetrics(boost::any_cast<InspectingMetricsPair>(any));
			break;
		}
		default:
		{
			APLOG_ERROR << "Unspecified Content: " << static_cast<int>(content);
			break;
		}
		}
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad Any Cast for Content " << static_cast<int>(content) << ": "
				<< err.what();
	} catch (std::runtime_error& err)
	{
		APLOG_ERROR << "Runtime Error in Conversion: " << err.what();
	}
}

void
FlightAnalysisDataHandling::collectAndSendInspectingMetrics(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	APLOG_DEBUG << "Collect and Send Inspecting Metrics.";

	auto steadyStateAnalysis = steadyStateAnalysisHandle_.get();

	if (!steadyStateAnalysis)
	{
		APLOG_ERROR << "Steady State Analysis Missing. Cannot Collect or Send Inspecting Metrics.";
		return;
	}

	SteadyStateMetrics inspectingMetrics = steadyStateAnalysis->getInspectingMetrics();
	Packet packet = dp->serialize(inspectingMetrics, Content::INSPECTING_METRICS);
	publisher_.publish(packet);
}

void
FlightAnalysisDataHandling::setInspectingMetrics(InspectingMetricsPair pair)
{
	auto steadyStateAnalysis = steadyStateAnalysisHandle_.get();

	if (!steadyStateAnalysis)
	{
		APLOG_ERROR << "Steady State Analysis Missing. Cannot Set Inspection Metrics.";
		return;
	}

	steadyStateAnalysis->setInspectingMetrics(pair);
}
