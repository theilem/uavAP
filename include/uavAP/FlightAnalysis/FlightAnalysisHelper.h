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
 * FlightAnalysisHelper.h
 *
 *  Created on: Jul 22, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_
#define UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_

#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"
#include "uavAP/Core/Framework/Helper.h"
#include "uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h"
#include "uavAP/FlightAnalysis/ManeuverAnalysis/ManeuverAnalysis.h"
#include "uavAP/FlightAnalysis/TrimAnalysis/TrimAnalysis.h"
#include "uavAP/FlightAnalysis/DataHandling/FlightAnalysisDataHandling.h"

class FlightAnalysisHelper : public Helper
{
public:
	FlightAnalysisHelper()
	{
		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefaultCreator<DataPresentation>();
		addDefaultConfigurable<IPC>();
		addCreator<SteadyStateAnalysis>();
		addCreator<ManeuverAnalysis>();
		addDefaultCreator<TrimAnalysis>();
		addDefaultCreator<FlightAnalysisDataHandling>();
	}
};

#endif /* UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_ */
