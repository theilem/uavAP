/*
 * FlightAnalysisHelper.h
 *
 *  Created on: Jul 22, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_
#define UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_



#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include "uavAP/FlightAnalysis/ManeuverPlanner/ManeuverPlanner.h"
#include <uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h>

using FlightAnalysisDefaults = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		IPC,
		DataPresentation,
		SignalHandler,
		SensingActuationIO>;

using FlightAnalysisHelper = StaticHelper<FlightAnalysisDefaults,
		DataHandling,
		ManeuverPlanner
>;

#endif /* UAVAP_FLIGHTANALYSIS_FLIGHTANALYSISHELPER_H_ */
