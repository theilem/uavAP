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
 * ManeuverPlanner.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: simonyu
 */


#include <vector>

#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/RectanguloidCondition.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>

ManeuverPlanner::ManeuverPlanner() :
		maneuverAnalysis_(false), maneuverAnalysisStatus_(), trimAnalysis_(false), overrideInterrupted_(
				false), manualActive_(false), maneuverActive_(false), lastManualActive_(false), lastManeuverActive_(
				false), manualRestart_(false), maneuverRestart_(false), overrideSeqNr_(0)
{
	controllerOutputOffset_.throttleOutput = 0;
}

std::shared_ptr<ManeuverPlanner>
ManeuverPlanner::create(const Configuration& config)
{
	auto maneuverPlanner = std::make_shared<ManeuverPlanner>();

	if (!maneuverPlanner->configure(config))
	{
		CPSLOG_ERROR << "ManeuverPlanner: Failed to Load Config.";
	}

	return maneuverPlanner;
}

bool
ManeuverPlanner::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Configuration maneuverSetTree;

	params.configure(pm);
	if (pm.map())
	{
		manualRestart_ = params.manualRestart();
		maneuverRestart_ = params.maneuverRestart();

		safetyCondition_ = std::make_shared<RectanguloidCondition>(params.safetyBounds());
	}

	if (pm.add("maneuvers", maneuverSetTree, false))
	{
		for (auto& setIt : maneuverSetTree)
		{
			ManeuverSet maneuverSet;

			for (auto& manIt : setIt.second)
			{
				Maneuver maneuver;
				maneuver.configure(manIt.second);
				maneuverSet.push_back(maneuver);
			}

			maneuverSetMap_.insert(std::make_pair(setIt.first, maneuverSet));
		}

		currentManeuverSet_ = maneuverSetMap_.end();
		currentManeuver_ = std::nullopt;
	}

	return pm.map();
}

bool
ManeuverPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, ConditionManager, DataPresentation>())
		{
			CPSLOG_ERROR << "ManeuverPlanner: Missing dependencies.";
			return true;
		}

		auto ipc = get<IPC>();

		overridePublisher_ = ipc->publishPackets("override");
		advancedControlPublisher_ = ipc->publish<AdvancedControl>("advanced_control_maneuver");
		maneuverAnalysisPublisher_ = ipc->publish<bool>("maneuver_analysis");
		maneuverAnalysisStatusPublisher_ = ipc->publishPackets("maneuver_analysis_status");
		trimAnalysisPublisher_ = ipc->publish<bool>("trim_analysis");

		break;
	}
	case RunStage::NORMAL:
	{

		auto ipc = get<IPC>();

		controllerOutputSubscription_ = ipc->subscribe<ControllerOutput>("controller_output",
				std::bind(&ManeuverPlanner::onControllerOutput, this, std::placeholders::_1));

		if (!controllerOutputSubscription_.connected())
		{
			CPSLOG_DEBUG << "ManeuverPlanner: Controller Output Subscription Missing.";
		}

		controllerOutputTrimSubscription_ = ipc->subscribe<ControllerOutput>(
				"controller_output_trim",
				std::bind(&ManeuverPlanner::onControllerOutputTrim, this, std::placeholders::_1));

		if (!controllerOutputTrimSubscription_.connected())
		{
			CPSLOG_DEBUG << "ManeuverPlanner: Controller Output Trim Subscription Missing.";
		}

		advancedControlSubscription_ = ipc->subscribe<AdvancedControl>("advanced_control",
				std::bind(&ManeuverPlanner::onAdvancedControl, this, std::placeholders::_1));

		if (!advancedControlSubscription_.connected())
		{
			CPSLOG_DEBUG << "ManeuverPlanner: Advanced Control Subscription Missing.";
		}

		if (params.useSafetyBounds())
		{
			auto conditionManager = get<ConditionManager>();

			conditionManager->activateCondition(safetyCondition_,
					std::bind(&ManeuverPlanner::safetyTrigger, this, std::placeholders::_1));
		}

		break;
	}
	case RunStage::FINAL:
	{
		std::unique_lock<std::mutex> maneuverAnalysisLock(maneuverAnalysisMutex_);
		bool maneuverAnalysis = maneuverAnalysis_;
		maneuverAnalysisLock.unlock();

		std::unique_lock<std::mutex> maneuverAnalysisStatusLock(maneuverAnalysisStatusMutex_);
		ManeuverAnalysisStatus maneuverAnalysisStatus = maneuverAnalysisStatus_;
		maneuverAnalysisStatusLock.unlock();

		std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
		bool trimAnalysis = trimAnalysis_;
		trimAnalysisLock.unlock();

		auto dp = get<DataPresentation>();

		maneuverAnalysisPublisher_.publish(maneuverAnalysis);
		maneuverAnalysisStatusPublisher_.publish(dp->serialize(maneuverAnalysisStatus));
		trimAnalysisPublisher_.publish(trimAnalysis);

		break;
	}
	default:
		break;
	}
	return false;
}

void
ManeuverPlanner::setManualOverride(const Override& override)
{
	if (overrideInterrupted_)
	{
		CPSLOG_WARN << "Override Interrupted.";
		return;
	}

	stopOverride();
	stopManeuverAnalysis();

	std::unique_lock<std::mutex> overrideLock(overrideMutex_);
	override_ = override;
	overrideLock.unlock();

	setOverride("", !override.isEmpty(), false);
	startOverride();
}

void
ManeuverPlanner::setManeuverOverride(const std::string& maneuverSet)
{
	if (overrideInterrupted_)
	{
		CPSLOG_WARN << "Override Interrupted.";
		return;
	}

	currentManeuverSet_ = maneuverSetMap_.find(maneuverSet);

	if (currentManeuverSet_ == maneuverSetMap_.end())
	{
		CPSLOG_ERROR << "ManeuverPlanner: Maneuver Set " << maneuverSet << " Not Found.";
		return;
	}

	stopOverride();
	stopManeuverAnalysis();

	currentManeuver_ = currentManeuverSet_->second.begin();

	setOverride(maneuverSet, false, true);
	startManeuverAnalysis();
	startOverride();
}

void
ManeuverPlanner::setControllerOutputOffset(const ControllerOutput& offset)
{
	std::unique_lock<std::mutex> lock(controllerOutputOffsetMutex_);
	controllerOutputOffset_ = offset;
	lock.unlock();
}

void
ManeuverPlanner::interruptOverride()
{
	if (overrideInterrupted_)
	{
		return;
	}

	overrideInterrupted_ = true;

	CPSLOG_DEBUG << "Interrupt Override.";

	std::unique_lock<std::mutex> overrideLock(overrideMutex_);
	lastManualOverride_ = override_;
	overrideLock.unlock();

	lastManeuver_ = currentManeuver_;
	lastManeuverSet_ = maneuverSet_;
	lastManualActive_ = manualActive_;
	lastManeuverActive_ = maneuverActive_;

	stopOverride();
}

void
ManeuverPlanner::resumeOverride()
{
	if (!overrideInterrupted_)
	{
		return;
	}

	CPSLOG_DEBUG << "Resume Override.";

	if (lastManualActive_ && manualRestart_)
	{
		std::unique_lock<std::mutex> overrideLock(overrideMutex_);
		override_ = lastManualOverride_;
		overrideLock.unlock();

		CPSLOG_DEBUG << "Resume Manual Override.";

		setOverride("", !lastManualOverride_.isEmpty(), false);
		startOverride();
	}
	else if (lastManeuverActive_ && maneuverRestart_)
	{
		maneuverSet_ = lastManeuverSet_;
		currentManeuver_ = lastManeuver_;

		CPSLOG_DEBUG << "Resume Maneuver Override.";

		setOverride(maneuverSet_, false, true);
		startOverride();
	}

	overrideInterrupted_ = false;
}

Override
ManeuverPlanner::getOverride() const
{
	std::lock_guard<std::mutex> lg(overrideMutex_);
	return override_;
}

unsigned int
ManeuverPlanner::getOverrideNr() const
{
	std::lock_guard<std::mutex> lg(overrideSeqNrMutex_);
	return overrideSeqNr_;
}

Rectanguloid
ManeuverPlanner::getSafetyBounds() const
{
	return params.safetyBounds();
}

ControllerOutput
ManeuverPlanner::getControllerOutputTrim() const
{
	std::unique_lock<std::mutex> trimLock(controllerOutputTrimMutex_);
	ControllerOutput controllerOutputTrim = controllerOutputTrim_;
	trimLock.unlock();

	std::unique_lock<std::mutex> offsetLock(controllerOutputOffsetMutex_);
	ControllerOutput controllerOutputOffset = controllerOutputOffset_;
	offsetLock.unlock();

	controllerOutputTrim.rollOutput += controllerOutputOffset.rollOutput;
	controllerOutputTrim.pitchOutput += controllerOutputOffset.pitchOutput;
	controllerOutputTrim.yawOutput += controllerOutputOffset.yawOutput;
	controllerOutputTrim.throttleOutput += controllerOutputOffset.throttleOutput;

	return controllerOutputTrim;
}

void
ManeuverPlanner::setOverride(const std::string& maneuverSet, const bool& manualActive,
		const bool& maneuverActive)
{
	maneuverSet_ = maneuverSet;
	manualActive_ = manualActive;
	maneuverActive_ = maneuverActive;
}

void
ManeuverPlanner::startManeuverAnalysis()
{
	std::unique_lock<std::mutex> maneuverAnalysisLock(maneuverAnalysisMutex_);
	maneuverAnalysis_ = true;
	bool maneuverAnalysis = maneuverAnalysis_;
	maneuverAnalysisLock.unlock();

	maneuverAnalysisPublisher_.publish(maneuverAnalysis);
}

void
ManeuverPlanner::stopManeuverAnalysis()
{
	std::unique_lock<std::mutex> maneuverAnalysisLock(maneuverAnalysisMutex_);
	maneuverAnalysis_ = false;
	bool maneuverAnalysis = maneuverAnalysis_;
	maneuverAnalysisLock.unlock();

	maneuverAnalysisPublisher_.publish(maneuverAnalysis);
}

void
ManeuverPlanner::startOverride()
{
	CPSLOG_DEBUG << "Start Override.";

	if (manualActive_)
	{
		CPSLOG_DEBUG << "Start Manual Override.";

		std::unique_lock<std::mutex> overrideLock(overrideMutex_);
		Override override = override_;
		overrideLock.unlock();

		std::unique_lock<std::mutex> maneuverAnalysisStatusLock(maneuverAnalysisStatusMutex_);
		maneuverAnalysisStatus_.reset();
		ManeuverAnalysisStatus maneuverAnalysisStatus = maneuverAnalysisStatus_;
		maneuverAnalysisStatusLock.unlock();

		std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
		trimAnalysis_ = false;
		bool trimAnalysis = trimAnalysis_;
		trimAnalysisLock.unlock();

		auto dp = get<DataPresentation>();
		if (!dp)
		{
			CPSLOG_ERROR << "DataPresentation missing, cannot send maneuver analysis";
			return;
		}

		overridePublisher_.publish(dp->serialize(override));
		maneuverAnalysisStatusPublisher_.publish(dp->serialize(maneuverAnalysisStatus));
		trimAnalysisPublisher_.publish(trimAnalysis);

		std::unique_lock<std::mutex> overrideSeqNrLock(overrideSeqNrMutex_);
		overrideSeqNr_++;
		overrideSeqNrLock.unlock();
	}
	else if (maneuverActive_)
	{
		CPSLOG_DEBUG << "Start Maneuver Override.";

		if (currentManeuver_ == currentManeuverSet_->second.end())
		{
			stopOverride();
			stopManeuverAnalysis();

			return;
		}

		activateManeuverOverride(std::bind(&ManeuverPlanner::nextManeuverOverride, this));
	}
}

void
ManeuverPlanner::stopOverride()
{
	CPSLOG_DEBUG << "Stop Override.";

	if (maneuverActive_)
	{
		deactivateManeuverOverride();
	}

	setOverride("", false, false);

	std::unique_lock<std::mutex> overrideLock(overrideMutex_);
	Override override;
	override_ = override;
	overrideLock.unlock();

	std::unique_lock<std::mutex> advancedControlLock(advancedControlMutex_);
	AdvancedControl advancedControl = advancedControl_;
	advancedControlLock.unlock();

	std::unique_lock<std::mutex> maneuverAnalysisStatusLock(maneuverAnalysisStatusMutex_);
	maneuverAnalysisStatus_.reset();
	ManeuverAnalysisStatus maneuverAnalysisStatus = maneuverAnalysisStatus_;
	maneuverAnalysisStatusLock.unlock();

	std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
	trimAnalysis_ = false;
	bool trimAnalysis = trimAnalysis_;
	trimAnalysisLock.unlock();

	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "DataPresentation missing, cannot send maneuver analysis";
		return;
	}

	overridePublisher_.publish(dp->serialize(override));
	advancedControlPublisher_.publish(advancedControl);
	maneuverAnalysisStatusPublisher_.publish(dp->serialize(maneuverAnalysisStatus));
	trimAnalysisPublisher_.publish(trimAnalysis);

	std::unique_lock<std::mutex> overrideSeqNrLock(overrideSeqNrMutex_);
	overrideSeqNr_++;
	overrideSeqNrLock.unlock();
}

void
ManeuverPlanner::nextManeuverOverride()
{
	if (!maneuverActive_)
	{
		return;
	}

	CPSLOG_DEBUG << "Next Maneuver Override.";

	deactivateManeuverOverride();

	if (currentManeuver_)
	{
		(*currentManeuver_)++;
	}
	else
	{
		stopOverride();
		stopManeuverAnalysis();
		return;
	}

	startOverride();
}

void
ManeuverPlanner::activateManeuverOverride(const ICondition::ConditionTrigger& conditionTrigger)
{
	if (!maneuverActive_)
	{
		return;
	}

	CPSLOG_DEBUG << "Activate Maneuver Override.";

	if (!currentManeuver_)
	{
		CPSLOG_WARN << "ManeuverPlanner: No Maneuver to Activate.";
		return;
	}

	auto& currentManeuver = *currentManeuver_;

	if (currentManeuver == currentManeuverSet_->second.end())
	{
		CPSLOG_WARN << "ManeuverPlanner: Maneuver Ended.";
		return;
	}

	if (currentManeuver->condition)
	{
		auto conditionManager = get<ConditionManager>();

		if (!conditionManager)
		{
			CPSLOG_ERROR << "ManeuverPlanner: Condition Manager Missing.";
			return;
		}

		conditionManager->activateCondition(currentManeuver->condition, conditionTrigger);
	}

	Override override = currentManeuver->override;
	bool controllerOutputOverrideFlag = currentManeuver->controllerOutputOverrideFlag;

	if (controllerOutputOverrideFlag)
	{
		ControllerOutputsOverrides controllerOutputOverrideType =
				currentManeuver->controllerOutputOverrideType;

		switch (controllerOutputOverrideType)
		{
		case ControllerOutputsOverrides::FREEZE:
		{
			std::unique_lock<std::mutex> lock(controllerOutputMutex_);
			ControllerOutput controllerOutputOverride = controllerOutput_;
			lock.unlock();

			std::map<ControllerOutputs, bool> controllerOutputOverrideMap =
					currentManeuver->controllerOutputOverrideMap;

			overrideControllerOutput(override, controllerOutputOverrideMap,
					controllerOutputOverride);

			break;
		}
		case ControllerOutputsOverrides::TRIM:
		{
			std::unique_lock<std::mutex> lock(controllerOutputTrimMutex_);
			ControllerOutput controllerOutputOverride = controllerOutputTrim_;
			lock.unlock();

			std::map<ControllerOutputs, bool> controllerOutputOverrideMap =
					currentManeuver->controllerOutputOverrideMap;

			overrideControllerOutput(override, controllerOutputOverrideMap,
					controllerOutputOverride);

			break;
		}
		case ControllerOutputsOverrides::OFFSET:
		{
			std::unique_lock<std::mutex> lock(controllerOutputTrimMutex_);
			ControllerOutput controllerOutputOverride = controllerOutputTrim_;
			lock.unlock();

			std::map<ControllerOutputs, bool> controllerOutputOverrideMap =
					currentManeuver->controllerOutputOverrideMap;
			std::map<ControllerOutputs, double> offsetOverride = override.output;
			ControllerOutput controllerOutputOffset;
			controllerOutputOffset.throttleOutput = 0;

			for (auto& it : offsetOverride)
			{
				switch (it.first)
				{
				case ControllerOutputs::ROLL:
				{
					controllerOutputOffset.rollOutput = it.second;
					break;
				}
				case ControllerOutputs::PITCH:
				{
					controllerOutputOffset.pitchOutput = it.second;
					break;
				}
				case ControllerOutputs::YAW:
				{
					controllerOutputOffset.yawOutput = it.second;
					break;
				}
				case ControllerOutputs::THROTTLE:
				{
					controllerOutputOffset.throttleOutput = it.second;
					break;
				}
				default:
				{
					break;
				}
				}
			}

			std::unique_lock<std::mutex> offsetLock(controllerOutputOffsetMutex_);
			controllerOutputOffset_ = controllerOutputOffset;
			offsetLock.unlock();

			overrideControllerOutput(override, controllerOutputOverrideMap,
					controllerOutputOverride);

			break;
		}
		default:
		{
			break;
		}
		}
	}

	std::unique_lock<std::mutex> overrideLock(overrideMutex_);
	override_ = override;
	overrideLock.unlock();

	AdvancedControl advancedControl = currentManeuver->advancedControl;

	CPSLOG_TRACE << "Maneuver Camber Control: "
			<< EnumMap<CamberControl>::convert(advancedControl.camberSelection);
	CPSLOG_TRACE << "Maneuver Special Control: "
			<< EnumMap<SpecialControl>::convert(advancedControl.specialSelection);
	CPSLOG_TRACE << "Maneuver Throw Control: "
			<< EnumMap<ThrowsControl>::convert(advancedControl.throwsSelection);
	CPSLOG_TRACE << "Maneuver Camber Value: " << advancedControl.camberValue;
	CPSLOG_TRACE << "Maneuver Special Value: " << advancedControl.specialValue;

	std::unique_lock<std::mutex> maneuverAnalysisStatusLock(maneuverAnalysisStatusMutex_);
	maneuverAnalysisStatus_.maneuver = maneuverSet_;
	maneuverAnalysisStatus_.analysis = currentManeuver->analyzeManeuver;
	maneuverAnalysisStatus_.interrupted = overrideInterrupted_;
	ManeuverAnalysisStatus maneuverAnalysisStatus = maneuverAnalysisStatus_;
	maneuverAnalysisStatusLock.unlock();

	std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
	trimAnalysis_ = currentManeuver->analyzeTrim;
	bool trimAnalysis = trimAnalysis_;
	trimAnalysisLock.unlock();

	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "DataPresentation missing, cannot send maneuver analysis";
		return;
	}

	overridePublisher_.publish(dp->serialize(override));
	advancedControlPublisher_.publish(advancedControl);
	maneuverAnalysisStatusPublisher_.publish(dp->serialize(maneuverAnalysisStatus));
	trimAnalysisPublisher_.publish(trimAnalysis);

	std::unique_lock<std::mutex> overrideSeqNrLock(overrideSeqNrMutex_);
	overrideSeqNr_++;
	overrideSeqNrLock.unlock();
}

void
ManeuverPlanner::deactivateManeuverOverride()
{
	if (!maneuverActive_)
	{
		return;
	}

	CPSLOG_DEBUG << "Deactivate Maneuver Override.";

	if (!currentManeuver_)
	{
		CPSLOG_WARN << "ManeuverPlanner: No Maneuver to Deactivate.";
		return;
	}

	auto& currentManeuver = *currentManeuver_;

	if (currentManeuver == currentManeuverSet_->second.end())
	{
		CPSLOG_WARN << "ManeuverPlanner: Maneuver Ended.";
		return;
	}

	if (currentManeuver->condition)
	{
		auto conditionManager = get<ConditionManager>();

		if (!conditionManager)
		{
			CPSLOG_ERROR << "ManeuverPlanner: Condition Manager Missing.";
			return;
		}

		conditionManager->deactivateCondition(currentManeuver->condition);
	}

	std::unique_lock<std::mutex> maneuverAnalysisStatusLock(maneuverAnalysisStatusMutex_);
	maneuverAnalysisStatus_.analysis = false;
	ManeuverAnalysisStatus maneuverAnalysisStatus = maneuverAnalysisStatus_;
	maneuverAnalysisStatusLock.unlock();

	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "DataPresentation missing, cannot send maneuver analysis";
		return;
	}

	maneuverAnalysisStatusPublisher_.publish(dp->serialize(maneuverAnalysisStatus));

	std::unique_lock<std::mutex> trimAnalysisLock(trimAnalysisMutex_);
	trimAnalysis_ = false;
	bool trimAnalysis = trimAnalysis_;
	trimAnalysisLock.unlock();

	trimAnalysisPublisher_.publish(trimAnalysis);
}

void
ManeuverPlanner::safetyTrigger(int trigger)
{
	if ((trigger == RectanguloidCondition::ENTER_RECTANGULOID && params.performInSafetyBounds())
			|| (trigger == RectanguloidCondition::EXIT_RECTANGULOID
					&& !params.performInSafetyBounds()))
	{
		resumeOverride();
	}
	else
	{
		interruptOverride();
	}
}

void
ManeuverPlanner::overrideControllerOutput(Override& override,
		const std::map<ControllerOutputs, bool>& overrideMap,
		const ControllerOutput& outputOverride)
{
	std::unique_lock<std::mutex> offsetLock(controllerOutputOffsetMutex_);
	ControllerOutput controllerOutputOffset = controllerOutputOffset_;
	offsetLock.unlock();

	for (auto& it : overrideMap)
	{
		if (it.second)
		{
			if (auto pair = findInMap(override.output, it.first))
			{
				switch (it.first)
				{
				case ControllerOutputs::ROLL:
				{
					pair->second = outputOverride.rollOutput;
					pair->second += controllerOutputOffset.rollOutput;
					break;
				}
				case ControllerOutputs::PITCH:
				{
					pair->second = outputOverride.pitchOutput;
					pair->second += controllerOutputOffset.pitchOutput;
					break;
				}
				case ControllerOutputs::YAW:
				{
					pair->second = outputOverride.yawOutput;
					pair->second += controllerOutputOffset.yawOutput;
					break;
				}
				case ControllerOutputs::THROTTLE:
				{
					pair->second = outputOverride.throttleOutput;
					pair->second += controllerOutputOffset.throttleOutput;
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
	}
}

void
ManeuverPlanner::onControllerOutput(const ControllerOutput& controllerOutput)
{
	Lock lock(controllerOutputMutex_);
	controllerOutput_ = controllerOutput;
	lock.unlock();
}

void
ManeuverPlanner::onControllerOutputTrim(const ControllerOutput& controllerOutputTrim)
{
	Lock lock(controllerOutputTrimMutex_);
	controllerOutputTrim_ = controllerOutputTrim;
	lock.unlock();
}

void
ManeuverPlanner::onAdvancedControl(const AdvancedControl& advanced)
{
	if (!maneuverActive_)
	{
		advancedControlPublisher_.publish(advanced);
	}
	else
	{
		std::unique_lock<std::mutex> lock(advancedControlMutex_);
		advancedControl_ = advanced;
		lock.unlock();
	}
}
