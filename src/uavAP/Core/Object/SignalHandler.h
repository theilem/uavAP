/*
 * SignalHandler.h
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_SIGNALHANDLER_H_
#define UAVAP_CORE_OBJECT_SIGNALHANDLER_H_

#include <boost/signals2.hpp>
#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/AggregatableObject.hpp>
#include <uavAP/Core/Time.h>
#include <csignal>
#include <thread>

class IScheduler;

void
sigIntHandler(int sig);

class SignalHandlerSingleton
{

public:

	~SignalHandlerSingleton()
	{
		if (!signalHandled_)
			joinHandler();
	}

	static SignalHandlerSingleton&
	getInstance()
	{
		static SignalHandlerSingleton instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	using OnSIGINT = boost::signals2::signal<void(int)>;
	using OnExit = boost::signals2::signal<void(void)>;

	void
	subscribeOnSigint(const OnSIGINT::slot_type& slot)
	{
		onSigint_.connect(slot);
	}

	void
	subscribeOnExit(const OnExit::slot_type& slot)
	{
		onExit_.connect(slot);
	}

	void
	callSigHandlers(int sig)
	{
		Lock lock(signalMutex_);
		APLOG_DEBUG << "SignalHandlerSingleton: Calling signal handlers";
		onSigint_(sig);
		onSigint_.disconnect_all_slots(); //Avoid double call

		APLOG_DEBUG << "SignalHandlerSingleton: Calling on exit";
		onExit_();
		onExit_.disconnect_all_slots(); //Avoid double call

		signalHandled_ = true;
	}

	void
	thisThreadBlockSigInt()
	{
		sigset_t set;
		sigaddset(&set, SIGINT);
		pthread_sigmask(SIG_SETMASK, &set, NULL);
	}

	void
	joinHandler()
	{
		signalHandled_ = true;
		if (signalHandlerThread_.joinable())
			signalHandlerThread_.join();
	}

	SignalHandlerSingleton(SignalHandlerSingleton const&) = delete;

	void
	operator=(SignalHandlerSingleton const&) = delete;

private:

	SignalHandlerSingleton() : signalHandled_(false)
	{
		thisThreadBlockSigInt();
		APLOG_TRACE << "SignalHandlerSingleton: Subscribe on SIGINT";

		signalHandlerThread_ = std::thread(std::bind(&SignalHandlerSingleton::signalHandleThreadTask, this));
//		signalHandlerThread_.detach();
//		std::signal(SIGTERM, sigIntHandler);
	}

	void
	signalHandleThreadTask()
	{
		sigset_t set;
		sigaddset(&set, SIGINT);
		pthread_sigmask(SIG_UNBLOCK, &set, NULL);
		std::signal(SIGINT, sigIntHandler);

		while (!signalHandled_)
			std::this_thread::sleep_for(Milliseconds(100));

		APLOG_DEBUG << "Signal handled. Exit.";

//		::exit(SIGINT);
	}

	Mutex signalMutex_;
	OnSIGINT onSigint_;
	OnExit onExit_;
	std::thread signalHandlerThread_;
	bool signalHandled_;

};

class SignalHandler: public AggregatableObject<IScheduler, SignalHandler>
{

public:

	SignalHandler(Aggregator& agg);

	static constexpr TypeId typeId = "signal_handler";

	using OnSIGINT = boost::signals2::signal<void(int)>;

	void
	subscribeOnSigint(const OnSIGINT::slot_type& slot);

private:

	void
	onExit();

	Aggregator& agg_;

};

#endif /* UAVAP_CORE_OBJECT_SIGNALHANDLER_H_ */
