/*
 * FlightControlHelper.h
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_
#define UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_

#include "uavAP/API/ChannelMixing.h"
#include "uavAP/API/Simulation/Connector/SimulationConnector.h"
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/DataPresentation/DataPresentationFactory.h"
#include "uavAP/Core/Framework/Helper.h"
#include "uavAP/Core/IDC/IDCFactory.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"

class SimulationHelper: public Helper
{
public:
	SimulationHelper()
	{
		addCreator<SimulationConnector>("connector");
		addCreator<ChannelMixing>("channel_mixing");

		addDefault<SchedulerFactory>("scheduler");
		addDefault<TimeProviderFactory>("time_provider");
		addDefaultCreator<IPC>("ipc");
		addDefault<IDCFactory>("idc");
		addDefault<DataPresentationFactory<Content,Target>>("data_presentation");
	}
};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_ */
