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
 * Geofencing.h
 *
 *  Created on: Aug 20, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCING_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCING_H_
#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/Object/AggregatableObject.hpp>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include <uavAP/MissionControl/Geofencing/ConstRollRateModel.h>
#include <uavAP/MissionControl/Geofencing/GeofencingParams.h>
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <uavAP/MissionControl/Polygon.h>

class IPC;
class IScheduler;
class ManeuverPlanner;
class IGeofencingModel;
class WindAnalysis;

class Geofencing: public AggregatableObject<IPC, IScheduler, ManeuverPlanner, IGeofencingModel, WindAnalysis>,
		public ConfigurableObject<GeofencingParams>,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "geofencing";

	Geofencing();

	bool
	run(RunStage stage) override;

	std::vector<Waypoint>
	getCriticalPoints() const;

private:

	void
	onSensorData(const SensorData& data);

	void
	evaluateSafety();

	SensorData sensorData_;
	WindInfo windEstimate_;
	Mutex sensorDataMutex_;

	mutable Mutex criticalPointsMutex_;
	std::vector<Waypoint> criticalPointsLeft_;
	std::vector<Waypoint> criticalPointsRight_;

	bool leftSafe_;
	bool rightSafe_;
	bool safetyActiveLeft_;
	bool safetyActiveRight_;

	Polygon geoFence_;
};

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCING_H_ */
