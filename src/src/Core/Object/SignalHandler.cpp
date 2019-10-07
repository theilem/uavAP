/*
 * SignalHandler.cpp
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include <uavAP/Core/Object/SignalHandler.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <csignal>
#include <cstdlib>

void
sigIntHandler(int sig)
{
	SignalHandlerSingleton::getInstance().callSigHandlers(sig);
	APLogger::instance()->flush(); //Synchronize stdio
//	::exit(sig);
}

SignalHandler::SignalHandler(Aggregator& agg) :
		agg_(agg)
{
	APLOG_DEBUG << "Signal handler created";
	SignalHandlerSingleton::getInstance().subscribeOnExit(std::bind(&SignalHandler::onExit, this));
}

void
SignalHandler::subscribeOnSigint(const OnSIGINT::slot_type& slot)
{
	APLOG_DEBUG << "SignalHandler: Subscribing on sigint";
	SignalHandlerSingleton::getInstance().subscribeOnSigint(slot);
}

void
SignalHandler::onExit()
{
	if (auto sched = get<IScheduler>())
		sched->stop();
//
//	APLOG_DEBUG << "Clear aggregator.";
//	auto sh = get<SignalHandler>(); // Get this object to avoid segfault when clearing the aggregator
//	agg_.clear();
//	APLOG_DEBUG << "Cleared aggregator.";
}
