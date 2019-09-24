/*
 * PubOptions.h
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_IPC_IPCOPTIONS_H_
#define UAVAP_CORE_IPC_IPCOPTIONS_H_

#include <functional>

class Subscription;

struct IPCOptions
{
	bool multiTarget = true;
	bool retry = false;

	std::function<void(const Subscription&)> retrySuccessCallback;

};



#endif /* UAVAP_CORE_IPC_IPCOPTIONS_H_ */
