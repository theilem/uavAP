////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * EmulationInterfaceHelper.h
 *
 *  Created on: Jan 22, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_

#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include "uavAP/Core/IDC/IDC.h"
#include "uavAP/Core/IDC/NetworkLayer/NetworkFactory.h"
#include "EduInterface.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"
#include "uavAP/Core/DataPresentation/DataPresentationFactory.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/Framework/Helper.h"

class EduInterfaceHelper: public Helper
{
public:
	EduInterfaceHelper()
	{
		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefault<DataPresentationFactory<Content, Target>>();
		addFactory<NetworkFactory>();

		addDefaultCreator<IDC>();

		addCreator<EduInterface>();
	}
};

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_ */
