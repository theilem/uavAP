/*
 * SignalHandler.cpp
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */
#include <uavAP/Core/Object/SignalHandler.h>
#include <csignal>
#include <cstdlib>

static SignalHandler* sigHandler = nullptr;

void
sigIntHandler(int sig)
{
	if (sigHandler)
	{
		sigHandler->callSigHandlers(sig);
	}
	exit(sig);
}

SignalHandler::SignalHandler()
{
	if (!sigHandler)
	{
		sigHandler = this;
		std::signal(SIGINT, sigIntHandler);
		std::signal(SIGTERM, sigIntHandler);
	}
}

void
SignalHandler::callSigHandlers(int sig)
{
	onSigint_(sig);
}

void
SignalHandler::subscribeOnSigint(const OnSIGINT::slot_type& slot)
{
	onSigint_.connect(slot);
}
