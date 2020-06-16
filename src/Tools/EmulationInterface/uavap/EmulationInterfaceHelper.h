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

#include "EmulationAPInterface.h"

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/NetworkFactory.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
//
//class EmulationInterfaceHelper: public Helper
//{
//public:
//	EmulationInterfaceHelper()
//	{
//		addDefault<SchedulerFactory>();
//		addDefault<TimeProviderFactory>();
//		addDefault<NetworkFactory>();
//		addDefault<DataPresentation>();
//
//		addCreator<EmulationAPInterface>();
//	}
//};

using EmulationInterfaceHelper = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		NetworkFactory,
		DataPresentation,
		EmulationAPInterface,
		SignalHandler>;

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_ */
