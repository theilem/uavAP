/**
 *  @file         SerialComm.h
 *  @author Simon Yu
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_
#define UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_

#include <cpsCore/cps_object>

#include <uavAP/Communication/Comm/IDCComm/IDCCommParams.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <cpsCore/Utilities/Packet.h>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/IPC/Publisher.h>
#include <cpsCore/Utilities/IDC/IDCSender.h>
#include <cpsCore/Utilities/LockTypes.hpp>
#include "uavAP/Communication/Comm/IComm.h"

class DataPresentation;
class IDC;
class IPC;


class IDCComm: public IComm,
			   public AggregatableObject<IPC, IDC, DataPresentation>,
			   public ConfigurableObject<IDCCommParams>,
			   public IRunnableObject
{
public:

	static constexpr TypeId typeId = "idc";

	IDCComm();

	bool
	run(RunStage stage) override;

private:

	void
	sendPacket(const Packet& packet);

	void
	receivePacket(const Packet& packet);

	void
	subscribeCallback(const Subscription& sub, Target target);

	std::vector<Subscription> subscriptions_;
	std::vector<Publisher<Packet>> publishers_;

	Mutex senderMutex_;
	IDCSender sender_;
	bool senderAvailable_;
	boost::signals2::connection idcConnection_;
};

#endif /* UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_MM_H_ */
