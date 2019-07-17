/*
 * SignalHandler.h
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_SIGNALHANDLER_H_
#define UAVAP_CORE_OBJECT_SIGNALHANDLER_H_

#include <boost/signals2.hpp>
#include <uavAP/Core/Object/AggregatableObject.hpp>

void
sigIntHandler(int sig) __attribute__((noreturn));

class SignalHandler: public AggregatableObject<>
{

public:

	SignalHandler();

#ifndef NO_RTTI
	using OnSIGINT = boost::signals2::signal<void(int)>;

	void
	subscribeOnSigint(const OnSIGINT::slot_type& slot);
#endif

	void
	callSigHandlers(int sig);

private:

#ifndef NO_RTTI
	OnSIGINT onSigint_;
#endif
};

#endif /* UAVAP_CORE_OBJECT_SIGNALHANDLER_H_ */
