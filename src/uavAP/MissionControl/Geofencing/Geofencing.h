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
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/MissionControl/Geofencing/ConstRollRateModel.h>
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <uavAP/MissionControl/Polygon.h>

class IPC;
class IScheduler;
class ManeuverPlanner;
class IGeofencingModel;

class Geofencing: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "geofencing";

	Geofencing();

	bool
	configure(const Configuration& config);

	ADD_CREATE_WITH_CONFIG(Geofencing);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	Mission
	criticalPoints();

private:

	void
	onSensorData(const SensorData& data);

	void
	evaluateSafety();

	SensorData sensorData_;
	std::mutex sensorDataMutex_;

	bool leftSafe_;
	bool rightSafe_;
	bool safetyActiveLeft_;
	bool safetyActiveRight_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<ManeuverPlanner> maneuverPlanner_;
	ObjectHandle<IGeofencingModel> geofencingModel_;

	double rollMax_;

	Polygon geoFence_;
	double evaluationThreshold_; //!< Threshold for Edge distance to be evaluated
	double distanceThreshold_;//!< Threshold for distance of evaluation to be considered as not safe
	Duration period_;

};

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_GEOFENCING_H_ */
