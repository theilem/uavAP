//
// Created by mirco on 26.02.21.
//

#include "uavAP/FlightAnalysis/ManeuverPlanner/ManeuverPlanner.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/IPC/IPC.h>

bool
ManeuverPlanner::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing deps";
				return true;
			}

			auto ipc = get<IPC>();
			IPCOptions opts;
			opts.variableSize = true;
			overridePublisher_ = ipc->publish<Maneuver::Overrides>("overrides", opts);
			maintainsPublisher_ = ipc->publish<Maneuver::Maintains>("maintains", opts);
			break;
		}
		case RunStage::NORMAL:
		{
			auto dh = get<DataHandling<Content, Target>>();
			dh->addTriggeredStatusFunction<std::vector<std::string>, DataRequest>(
					[this](const DataRequest& req) -> Optional<std::vector<std::string>>
					{
						if (req == DataRequest::MANEUVERS_LIST)
						{
							std::vector<std::string> ids;
							for (const auto& it : params.maneuverSets())
								ids.push_back(it.first);
							return ids;
						}
						return std::nullopt;
					}, Content::MANEUVER_LIST, Content::REQUEST_DATA);

			dh->subscribeOnData<std::string>(Content::SELECT_MANEUVER_SET, [this](const auto& id)
			{ maneuverSelection(id); });
			dh->subscribeOnData<bool>(Content::ABORT_MANEUVER, [this](const auto& abort)
			{ if (abort) stopManeuver(); });


			auto sched = get<IScheduler>();
			sched->schedule([this]()
							{ checkManeuver(); }, Milliseconds(params.period()), Milliseconds(params.period()));

			break;
		}
		default:
			break;
	}
	return false;
}

void
ManeuverPlanner::maneuverSelection(const std::string& maneuverId)
{

	CPSLOG_DEBUG << "Maneuver selected: " << maneuverId;
	auto it = params.maneuverSets().find(maneuverId);
	if (it == params.maneuverSets().end())
	{
		CPSLOG_ERROR << "Selected maneuver " << maneuverId << " does not exist";
		return;
	}

	activeManeuverSet_ = &(*it);

	startManeuverSet();
}

void
ManeuverPlanner::startManeuverSet()
{
	if (activeManeuverSet_->second.saveAs())
	{
		auto io = get<ISensingIO>();
		maneuverLogFile_.open(params.logPath() + humanReadableTimeOfDay(io->getSensorData().timestamp) +
							  "_" + *activeManeuverSet_->second.saveAs());
		CPSLOG_DEBUG << "opening " << params.logPath() + *activeManeuverSet_->second.saveAs();
	}
	activeManeuver_ = activeManeuverSet_->second.maneuvers().begin();

	maneuver_ = createManeuver(*activeManeuver_);
	CPSLOG_DEBUG << "Starting ManeuverSet " << activeManeuverSet_->first;
	activateManeuver();
}

std::shared_ptr<Maneuver>
ManeuverPlanner::createManeuver(const ManeuverParams& p)
{
	auto man = std::make_shared<Maneuver>();
	man->setParams(p);
	man->initialize(*aggregator_);
	return man;
}

void
ManeuverPlanner::activateManeuver()
{

	overridePublisher_.publish(maneuver_->getOverrides());
	maintainsPublisher_.publish(maneuver_->getMaintains());
	maneuver_->printInfo();
	if (maneuverLogFile_.is_open())
	{
		auto io = get<ISensingIO>();
		maneuverLogFile_ << io->getSensorData().sequenceNumber << ",";
		CPSLOG_DEBUG << "Logging maneuver sequence number: " << io->getSensorData().sequenceNumber;
	}

}

void
ManeuverPlanner::checkManeuver()
{
	if (!maneuver_)
		return;

	if (maneuver_->inTransition())
	{
		CPSLOG_DEBUG << "Transitioning to next Maneuver";
		activeManeuver_++;
		if (activeManeuver_ == activeManeuverSet_->second.maneuvers().end())
		{
			overridePublisher_.publish(Maneuver::Overrides());
			maintainsPublisher_.publish(Maneuver::Maintains());
			maneuver_.reset();
			if (maneuverLogFile_.is_open())
			{
				auto io = get<ISensingIO>();
				maneuverLogFile_ << io->getSensorData().sequenceNumber;
				maneuverLogFile_.close();
			}

			CPSLOG_DEBUG << "ManeuverSet done";
			return;
		}

		maneuver_ = createManeuver(*activeManeuver_);
		activateManeuver();
	}
	else
	{
		if (maneuver_->isTimeVarying())
		{
			overridePublisher_.publish(maneuver_->getOverrides());
			maintainsPublisher_.publish(maneuver_->getMaintains());
		}
	}
}

void
ManeuverPlanner::stopManeuver()
{
	maneuver_.reset();
	overridePublisher_.publish(Maneuver::Overrides());
	maintainsPublisher_.publish(Maneuver::Maintains());
	activeManeuverSet_ = nullptr;
}
