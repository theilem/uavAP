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

#include "uavAP/FlightAnalysis/TrimAnalysis/TrimAnalysis.h"
#include "uavAP/Core/Object/AggregatableObjectImpl.hpp"
#include "uavAP/Core/IPC/IPC.h"

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
		if (!checkIsSet<IPC>())
		{
			APLOG_ERROR << "TrimAnalysis: Missing dependencies.";
			return true;
		}

		if (auto ipc = get<IPC>())
		{
			controllerOutputTrimPublisher_ = ipc->publish<ControllerOutput>("controller_output_trim");
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		controllerOutputSubscription_ = ipc->subscribe<ControllerOutput>("controller_output",
				std::bind(&TrimAnalysis::onControllerOutput, this, std::placeholders::_1));

		if (!controllerOutputSubscription_.connected())
		{
			APLOG_ERROR << "TrimAnalysis: Controller Output Subscription Missing.";
			return true;
		}

		trimAnalysisSubscription_ = ipc->subscribe<bool>("trim_analysis",
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
TrimAnalysis::onControllerOutput(const ControllerOutput& output)
{
	Lock trimAnalysisLock(trimAnalysisMutex_);
	bool trimAnalysis = trimAnalysis_;
	trimAnalysisLock.unlock();

	bool trimAnalysisLast = trimAnalysisLast_;

	Lock controllerOutputLock(controllerOutputMutex_);
	if (trimAnalysisLast == false && trimAnalysis == true)
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
	controllerOutputLock.unlock();

	trimAnalysisLast_ = trimAnalysis;
}

void
TrimAnalysis::onTrimAnalysis(const bool& analysis)
{
	Lock lock(trimAnalysisMutex_);
	trimAnalysis_ = analysis;
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
}

void
TrimAnalysis::analysisNormal(const ControllerOutput& controllerOutput)
{
	controllerOutputTrim_.rollOutput += controllerOutput.rollOutput;
	controllerOutputTrim_.pitchOutput += controllerOutput.pitchOutput;
	controllerOutputTrim_.yawOutput += controllerOutput.yawOutput;
	controllerOutputTrim_.throttleOutput += controllerOutput.throttleOutput;
	controllerOutputCount_.rollOutput ++;
	controllerOutputCount_.pitchOutput ++;
	controllerOutputCount_.yawOutput ++;
	controllerOutputCount_.throttleOutput ++;

	APLOG_DEBUG << "rollOutput: " << controllerOutputTrim_.rollOutput;
	APLOG_DEBUG << "pitchOutput: " << controllerOutputTrim_.pitchOutput;
	APLOG_DEBUG << "yawOutput: " << controllerOutputTrim_.yawOutput;
	APLOG_DEBUG << "throttleOutput: " << controllerOutputTrim_.throttleOutput;
	APLOG_DEBUG << "rollOutputCount: " << controllerOutputCount_.rollOutput;
	APLOG_DEBUG << "pitchOutputCount: " << controllerOutputCount_.pitchOutput;
	APLOG_DEBUG << "yawOutputCount: " << controllerOutputCount_.yawOutput;
	APLOG_DEBUG << "throttleOutputCount: " << controllerOutputCount_.throttleOutput;
	APLOG_DEBUG << "";
}

void
TrimAnalysis::analysisFinal()
{
	controllerOutputTrim_.rollOutput /= controllerOutputCount_.rollOutput;
	controllerOutputTrim_.pitchOutput /= controllerOutputCount_.pitchOutput;
	controllerOutputTrim_.yawOutput /= controllerOutputCount_.yawOutput;
	controllerOutputTrim_.throttleOutput /= controllerOutputCount_.throttleOutput;
	controllerOutputTrimPublisher_.publish(controllerOutputTrim_);
}
