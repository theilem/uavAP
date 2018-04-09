/*
 * EmulationInterfaceHelper.h
 *
 *  Created on: Jan 22, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_

#include "EduInterface.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/DataPresentation/DataPresentationFactory.h"
#include "uavAP/Core/IDC/IDCFactory.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/Framework/Helper.h"


class EduInterfaceHelper: public Helper
{
public:
	EduInterfaceHelper()
	{
		addDefault<SchedulerFactory>("scheduler");
		addDefault<TimeProviderFactory>("time_provider");
		addDefault<IDCFactory>("idc");
		addDefault<DataPresentationFactory<Content, Target>>("data_presentation");

		addCreator<EduInterface>("interface");
	}
};

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_ */
