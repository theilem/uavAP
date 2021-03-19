//
// Created by mirco on 23.02.21.
//

#include "uavAP/Core/OverrideHandler/OverrideHandler.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/Optional.hpp>
#include <cpsCore/Utilities/IPC/IPC.h>

void
OverrideHandler::disableAllOverrides()
{
	for (const auto&[id, handle]:overrideHandles_)
	{
		handle(false, 0);
	}
}

bool
OverrideHandler::applyOverride(const std::string& overrideId, bool enable, FloatingType value)
{
	if (!enabled_)
		return false;
	auto it = overrideHandles_.find(overrideId);
	if (it == overrideHandles_.end())
		return false;

	it->second(enable, value);

	return true;
}

void
OverrideHandler::registerOverride(const std::string& overrideId, const OverrideHandler::OverrideHandle& overrideHandle)
{
	if (overrideHandles_.find(overrideId) != overrideHandles_.end())
	{
		CPSLOG_ERROR << "Override with ID " << overrideId << " already registered";
		return;
	}
	overrideHandles_.insert(std::make_pair(overrideId, overrideHandle));
}

void
OverrideHandler::registerMaintain(const std::string& overrideId, const OverrideHandler::MaintainHandle& maintainHandle)
{
	if (maintainHandles_.find(overrideId) != maintainHandles_.end())
	{
		CPSLOG_ERROR << "Maintain with ID " << overrideId << " already registered";
		return;
	}
	maintainHandles_.insert(std::make_pair(overrideId, maintainHandle));

}


void
OverrideHandler::registerOverride(const std::string& overrideId, OverridableValue<FloatingType>& overridableValue)
{
	if (overrideHandles_.find(overrideId) != overrideHandles_.end())
	{
		CPSLOG_ERROR << "Override with ID " << overrideId << " already registered";
		return;
	}
	overrideHandles_.insert(std::make_pair(overrideId, [&overridableValue](bool enable, FloatingType value)
	{ overridableValue.applyOverride(enable, value); }));
}

void
OverrideHandler::registerOverride(const std::string& overrideId, MaintainableValue<FloatingType>& trimmableValue)
{
	if (overrideHandles_.find(overrideId) != overrideHandles_.end())
	{
		CPSLOG_ERROR << "Override with ID " << overrideId << " already registered";
		return;
	}
	overrideHandles_.insert(std::make_pair(overrideId, [&trimmableValue](bool enable, FloatingType value)
	{ trimmableValue.applyOverride(enable, value); }));

	if (maintainHandles_.find(overrideId) != maintainHandles_.end())
	{
		CPSLOG_ERROR << "Trim with ID " << overrideId << " already registered";
		return;
	}
	maintainHandles_.insert(std::make_pair(overrideId, [&trimmableValue](bool enable)
	{ trimmableValue.maintainValue(enable); }));
}

void
OverrideHandler::registerOverride(const std::string& overrideId,
								  OverridableValue<Angle<FloatingType>>& overridableValue)
{
	if (overrideHandles_.find(overrideId) != overrideHandles_.end())
	{
		CPSLOG_ERROR << "Override with ID " << overrideId << " already registered";
		return;
	}
	overrideHandles_.insert(std::make_pair(overrideId, [&overridableValue](bool enable, FloatingType value)
	{ overridableValue.applyOverride(enable, Angle<FloatingType>(value)); }));
}

void
OverrideHandler::registerOverride(const std::string& overrideId, MaintainableValue<Angle<FloatingType>>& trimmableValue)
{

	if (overrideHandles_.find(overrideId) != overrideHandles_.end())
	{
		CPSLOG_ERROR << "Override with ID " << overrideId << " already registered";
		return;
	}
	overrideHandles_.insert(std::make_pair(overrideId, [&trimmableValue](bool enable, FloatingType value)
	{ trimmableValue.applyOverride(enable, Angle<FloatingType>(value)); }));

	if (maintainHandles_.find(overrideId) != maintainHandles_.end())
	{
		CPSLOG_ERROR << "Trim with ID " << overrideId << " already registered";
		return;
	}
	maintainHandles_.insert(std::make_pair(overrideId, [&trimmableValue](bool enable)
	{ trimmableValue.maintainValue(enable); }));
}


bool
OverrideHandler::run(RunStage stage)
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

			auto dh = get<DataHandling>();
			dh->addTriggeredStatusFunction<std::vector<std::string>, DataRequest>(
					[this](const DataRequest& req) -> Optional<std::vector<std::string>>
					{
						if (req == DataRequest::OVERRIDE_LIST) return getPossibleOverrideIds();
						return std::nullopt;
					}, Content::OVERRIDE_LIST, Content::REQUEST_DATA);

			dh->subscribeOnData<std::map<std::string, FloatingType>>(Content::OVERRIDE,
																	 [this](const std::map<std::string, FloatingType>& overrides)
																	 {
																		 applyOverrides(overrides);
																	 });

			dh->subscribeOnData<std::vector<std::string>>(Content::MAINTAIN,
														  [this](const std::vector<std::string>& maintains)
														  {
															  applyMaintains(maintains);
														  });

			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();
			ipc->subscribe<std::map<std::string, FloatingType>>("overrides", [this](const auto& overrides)
			{ applyOverrides(overrides); });
			ipc->subscribe<std::vector<std::string>>("maintains", [this](const auto& maintains)
			{ applyMaintains(maintains); });
			break;
		}
		default:
			break;
	}
	return false;
}

std::vector<std::string>
OverrideHandler::getPossibleOverrideIds() const
{
	std::vector<std::string> ids;
	for (const auto&[id, handle]:overrideHandles_)
	{
		ids.push_back(id);
	}
	return ids;
}

void
OverrideHandler::applyOverrides(const std::map<std::string, FloatingType>& overrides)
{
	if (!enabled_)
		return;
	for (const auto&[id, handle]:overrideHandles_)
	{
		auto it = overrides.find(id);
		// If override not found, disable override
		if (it == overrides.end())
			handle(false, 0);
		else
			handle(true, it->second);
	}
}

void
OverrideHandler::applyMaintains(const std::vector<std::string>& maintains)
{
	if (!enabled_)
		return;
	for (const auto&[id, handle]:maintainHandles_)
	{
		auto it = std::find(maintains.begin(), maintains.end(), id);
		// If override not found, disable override
		if (it == maintains.end())
			handle(false);
		else
		{
			std::cout << "Applying maintain to " << id << std::endl;
			handle(true);
		}
	}
}

void
OverrideHandler::enable()
{
	enabled_ = true;
}


void
OverrideHandler::disable()
{
	disableAllOverrides();
	enabled_ = false;
}