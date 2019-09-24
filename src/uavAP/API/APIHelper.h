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
 * APIHelper.h
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */

#ifndef UAVAP_API_APIHELPER_H_
#define UAVAP_API_APIHELPER_H_
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>

class APIHelper: public Helper
{
public:

	APIHelper()
	{
		addDefault<TimeProviderFactory>();
		addDefault<SchedulerFactory>();
		addDefaultCreator<IPC>();
		addConfigurable<DataPresentation>();
	}
};

#endif /* UAVAP_API_APIHELPER_H_ */
