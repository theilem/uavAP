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
 * ManeuverAnalysis.h
 *
 *  Created on: Aug 6, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSIS_H_
#define UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSIS_H_

#include <string>
#include <iostream>
#include <fstream>

#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Object/AggregatableObject.hpp"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/EnumMap.hpp"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightAnalysis/ManeuverAnalysis/ManeuverAnalysisStatus.h"

enum class Maneuvers
{
	INVALID, GEOFENCING, ADVANCED_CONTROL, FLIGHT_TESTING, NUM_MANEUVERS
};

enum class CollectStates
{
	INVALID, INIT, NORMAL, FINAL, NUM_STATES
};

ENUMMAP_INIT(Maneuvers, { {Maneuvers::GEOFENCING, "geofencing"}, {Maneuvers::ADVANCED_CONTROL,
		"advanced_control"}, {Maneuvers::FLIGHT_TESTING, "flight_testing"}});

class IPC;
class IScheduler;
class DataPresentation;

class ManeuverAnalysis: public AggregatableObject<IPC, IScheduler, DataPresentation>,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "maneuver_analysis";

	ManeuverAnalysis();

	~ManeuverAnalysis();

	static std::shared_ptr<ManeuverAnalysis>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

private:

	void
	onSensorData(const SensorData& data);

	void
	onControllerOutput(const ControllerOutput& output);

	void
	onManeuverAnalysis(const bool& analysis);

	void
	onManeuverAnalysisStatus(const Packet& status);

	void
	collectStateInit(const SensorData& data, const std::string& maneuver, const bool& interrupted);

	void
	collectStateNormal(const SensorData& data, const bool& analysis);

	void
	collectStateFinal(const SensorData& data);

	void
	collectGeofencing(const SensorData& data, const CollectStates& states);

	void
	collectAdvancedControl(const SensorData& data, const CollectStates& states);

	void
	collectFlightTesting(const SensorData& data, const CollectStates& states);

	Subscription sensorDataSubscription_;
	Subscription controllerOutputSubscription_;
	Subscription maneuverAnalysisSubscription_;
	Subscription maneuverAnalysisStatusSubscription_;

	ControllerOutput controllerOutput_;
	Mutex controllerOutputMutex_;

	bool analysis_;
	Mutex maneuverAnalysisMutex_;

	ManeuverAnalysisStatus analysisStatus_;
	Mutex maneuverAnalysisStatusMutex_;

	bool collectInit_;
	unsigned counter_;
	Maneuvers maneuver_;

	std::string logPath_;
	std::ofstream logFile_;

	SensorData sensorData_;
	Mutex sensorDataMutex_;
};

#endif /* UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSIS_H_ */
