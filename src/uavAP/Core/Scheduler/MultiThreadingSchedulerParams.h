/*
 * MultiThreadingSchedulerParams.h
 *
 *  Created on: Aug 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULERPARAMS_H_
#define UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULERPARAMS_H_
#include <uavAP/Core/PropertyMapper/Parameter.h>


struct MultiThreadingSchedulerParams
{
	Parameter<size_t> priority = {20, "priority", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & priority;
	}
};


#endif /* UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULERPARAMS_H_ */
