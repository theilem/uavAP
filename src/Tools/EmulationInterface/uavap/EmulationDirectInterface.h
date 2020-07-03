//
// Created by mirco on 03.07.20.
//

#ifndef UAVAP_EMULATIONDIRECTINTERFACE_H
#define UAVAP_EMULATIONDIRECTINTERFACE_H

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IDC/IDCSender.h>
#include "EmulationDirectInterfaceParams.h"

class AggregatableAutopilotAPI;

class IDC;

class EmulationDirectInterface
		: public AggregatableObject<AggregatableAutopilotAPI, IDC>,
		  public IRunnableObject,
		  public ConfigurableObject<EmulationDirectInterfaceParams>
{
public:

	static constexpr TypeId typeId = "emulation_direct_interface";

	bool
	run(RunStage stage) override;

private:

	void
	onSensorDataPacket(const Packet& packet);

	IDCSender controlSender_;
};


#endif //UAVAP_EMULATIONDIRECTINTERFACE_H
