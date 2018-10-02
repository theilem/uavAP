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
 * DirectRollModel.h
 *
 *  Created on: Aug 15, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_DIRECTROLLMODEL_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_DIRECTROLLMODEL_H_

#include <acb.h>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Frames/VehicleOneFrame.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/MissionControl/Geofencing/IGeofencingModel.h"
#include "uavAP/MissionControl/Polygon.h"

class DirectRollModel: public IGeofencingModel, public IAggregatableObject
{
public:

	static constexpr const char* const typeId = "direct_roll";

	DirectRollModel();

	static std::shared_ptr<DirectRollModel>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	updateModel(const SensorData& data) override;

	std::vector<Vector3>
	getCriticalPoints(const Edge& edge, RollDirection dir) override;

private:

	double rollMax_;
	double g_;

	Vector3 centerOrbitLeft_;
	Vector3 centerOrbitRight_;
	double radiusOrbit_;
	std::mutex queryMutex_;
};

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_DIRECTROLLMODEL_H_ */
