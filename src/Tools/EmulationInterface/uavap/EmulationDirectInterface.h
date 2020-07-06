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
class DataPresentation;
struct ControllerOutput;

class EmulationDirectInterface
		: public AggregatableObject<AggregatableAutopilotAPI, IDC, DataPresentation>,
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

	void
	sendControllerOutput(const ControllerOutput& output);

	IDCSender controlSender_;
};


#endif //UAVAP_EMULATIONDIRECTINTERFACE_H
