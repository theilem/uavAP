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
 * TrimAnalysis.cpp
 *
 *  Created on: Mar 19, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/FlightAnalysis/TrimAnalysis/TrimAnalysis.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

TrimAnalysis::TrimAnalysis() :
		trimAnalysis_(false), trimAnalysisLast_(false)
{
}

std::shared_ptr<TrimAnalysis>
TrimAnalysis::create(const boost::property_tree::ptree& config)
{
	auto trimAnalysis = std::make_shared<TrimAnalysis>();

	if (!trimAnalysis->configure(config))
	{
		APLOG_ERROR << "TrimAnalysis: Failed to Load Config.";
	}

	return trimAnalysis;
}

bool
TrimAnalysis::configure(const boost::property_tree::ptree& config)
{
	return true;
}

bool
TrimAnalysis::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "TrimAnalysis: IPC Missing.";
			return true;
		}

		auto ipc = ipc_.get();

		controllerOutputTrimPublisher_ = ipc->publishPackets("controller_output_trim");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		controllerOutputSubscription_ = ipc->subscribeOnPacket("controller_output",
				std::bind(&TrimAnalysis::onControllerOutput, this, std::placeholders::_1));

		if (!controllerOutputSubscription_.connected())
		{
			APLOG_ERROR << "TrimAnalysis: Controller Output Subscription Missing.";
			return true;
		}

		trimAnalysisSubscription_ = ipc->subscribeOnPacket("trim_analysis",
				std::bind(&TrimAnalysis::onTrimAnalysis, this, std::placeholders::_1));

		if (!controllerOutputSubscription_.connected())
		{
			APLOG_ERROR << "TrimAnalysis: Controller Output Subscription Missing.";
			return true;
		}

		break;
	}
	case RunStage::FINAL:
	{
		analysisInit();
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
TrimAnalysis::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
}

void
TrimAnalysis::onControllerOutput(const Packet& output)
{
	std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
	bool trimAnalysis = trimAnalysis_;
	trimAnalysisLock.unlock();

	bool trimAnalysisLast = trimAnalysisLast_;

	std::unique_lock<std::mutex> controllerOutputLock(controllerOutputMutex_);
	if (trimAnalysisLast == false && trimAnalysis == false)
	{
		analysisIdle();
	}
	else if (trimAnalysisLast == false && trimAnalysis == true)
	{
		analysisInit();
	}
	else if (trimAnalysisLast == true && trimAnalysis == true)
	{
		analysisNormal(output);
	}
	else if (trimAnalysisLast == true && trimAnalysis == false)
	{
		analysisFinal();
	}

	APLOG_ERROR << "trimAnalysis: " << trimAnalysis;
	APLOG_ERROR << "trimAnalysisLast: " << trimAnalysisLast;
	APLOG_ERROR << "rollOutput: " << controllerOutputTrim_.rollOutput;
	APLOG_ERROR << "pitchOutput: " << controllerOutputTrim_.pitchOutput;
	APLOG_ERROR << "yawOutput: " << controllerOutputTrim_.yawOutput;
	APLOG_ERROR << "throttleOutput: " << controllerOutputTrim_.throttleOutput;
	APLOG_ERROR << "rollOutputCount: " << controllerOutputCount_.rollOutput;
	APLOG_ERROR << "pitchOutputCount: " << controllerOutputCount_.pitchOutput;
	APLOG_ERROR << "yawOutputCount: " << controllerOutputCount_.yawOutput;
	APLOG_ERROR << "throttleOutputCount: " << controllerOutputCount_.throttleOutput;
	APLOG_ERROR << "";
	controllerOutputLock.unlock();

	trimAnalysisLast_ = trimAnalysis;
}

void
TrimAnalysis::onTrimAnalysis(const Packet& analysis)
{
	std::unique_lock<std::mutex> lock(trimAnalysisMutex_);
	trimAnalysis_ = dp::deserialize<bool>(analysis);
	lock.unlock();
}

void
TrimAnalysis::analysisInit()
{
	controllerOutputTrim_.rollOutput = 0;
	controllerOutputTrim_.pitchOutput = 0;
	controllerOutputTrim_.yawOutput = 0;
	controllerOutputTrim_.throttleOutput = -1;
	controllerOutputCount_.rollOutput = 0;
	controllerOutputCount_.pitchOutput = 0;
	controllerOutputCount_.yawOutput = 0;
	controllerOutputCount_.throttleOutput = 0;
	controllerOutputTrimPublisher_.publish(dp::serialize(controllerOutputTrim_));
}

void
TrimAnalysis::analysisNormal(const Packet& output)
{
	ControllerOutput controllerOutput = dp::deserialize<ControllerOutput>(output);
	controllerOutputTrim_.rollOutput += controllerOutput.rollOutput;
	controllerOutputTrim_.pitchOutput += controllerOutput.pitchOutput;
	controllerOutputTrim_.yawOutput += controllerOutput.yawOutput;
	controllerOutputTrim_.throttleOutput += controllerOutput.throttleOutput;
	controllerOutputCount_.rollOutput ++;
	controllerOutputCount_.pitchOutput ++;
	controllerOutputCount_.yawOutput ++;
	controllerOutputCount_.throttleOutput ++;
}

void
TrimAnalysis::analysisFinal()
{
	controllerOutputTrim_.rollOutput /= controllerOutputCount_.rollOutput;
	controllerOutputTrim_.pitchOutput /= controllerOutputCount_.pitchOutput;
	controllerOutputTrim_.yawOutput /= controllerOutputCount_.yawOutput;
	controllerOutputTrim_.throttleOutput /= controllerOutputCount_.throttleOutput;
	controllerOutputTrimPublisher_.publish(dp::serialize(controllerOutputTrim_));
}

void
TrimAnalysis::analysisIdle()
{
	controllerOutputTrimPublisher_.publish(dp::serialize(controllerOutputTrim_));
}
