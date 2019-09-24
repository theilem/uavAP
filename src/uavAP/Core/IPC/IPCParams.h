/*
 * IPCParams.h
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_IPC_IPCPARAMS_H_
#define UAVAP_CORE_IPC_IPCPARAMS_H_
#include <uavAP/Core/PropertyMapper/Parameter.h>


struct IPCParams
{
	Parameter<std::size_t> maxPacketSize = {16384, "max_size", false};
	Parameter<std::size_t> maxNumPackets = {10, "max_num", false};
	Parameter<unsigned int> retryPeriod = {1000, "retry_period_ms", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & maxPacketSize;
		c & maxNumPackets;
		c & retryPeriod;
	}
};




#endif /* UAVAP_CORE_IPC_IPCPARAMS_H_ */
