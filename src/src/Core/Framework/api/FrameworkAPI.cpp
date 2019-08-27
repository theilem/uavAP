/*
 * FrameworkAPI.cpp
 *
 *  Created on: Feb 1, 2019
 *      Author: mirco
 */
#include <uavAP/Core/Framework/api/FrameworkAPI.h>

FrameworkAPI* FrameworkAPI::instance_ = nullptr;

FrameworkAPI*
FrameworkAPI::instance()
{
	static CGuard g;
	if (!instance_)
		instance_ = new FrameworkAPI;
	return instance_;
}

std::unique_lock<std::mutex>
FrameworkAPI::lockAggregator()
{
	std::unique_lock<std::mutex> lock(instance()->aggMutex_);
	return std::move(lock);
}

Aggregator*
FrameworkAPI::getAggregator()
{
	return &instance()->agg_;
}

FrameworkAPI::FrameworkAPI()
{
}

FrameworkAPI::CGuard::~CGuard()
{
	if (FrameworkAPI::instance_ != nullptr)
	{
		delete FrameworkAPI::instance_;
		FrameworkAPI::instance_ = nullptr;
	}
}
