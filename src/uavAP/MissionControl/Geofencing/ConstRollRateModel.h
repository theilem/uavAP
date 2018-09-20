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
 * ConstRollRateModel.h
 *
 *  Created on: Aug 15, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODEL_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODEL_H_

#include <acb.h>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Frames/VehicleOneFrame.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/MissionControl/Geofencing/IGeofencingModel.h"

class ConstRollRateModel: public IGeofencingModel, public IAggregatableObject
{
public:

	static constexpr const char* const typeId = "roll_rate";

	ConstRollRateModel();

	~ConstRollRateModel();

	static std::shared_ptr<ConstRollRateModel>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	updateModel(const SensorData& data) override;

	std::vector<Vector3>
	getCriticalPoints(const Edge& edge, RollDirection dir) override;

	/**
	 * @brief If frameleft/right is set it is calculating the position given a specified roll angle
	 * @param roll
	 * @param dir
	 * @return
	 */
	Vector3
	calculatePoint(double roll, RollDirection dir);

	/**
	 * @brief If frameleft/right is set it is calculating the yaw given a specified roll angle
	 * @param roll
	 * @param dir
	 * @return
	 */
	double
	calculateYaw(double roll, RollDirection dir);

	std::vector<double>
	calculateRoll(double yaw, RollDirection dir);

	Vector3
	toVector(const acb_t& complex);

private:

	double rollRate_;
	double rollMax_;
	long int precision_;
	double g_;

	double currentRoll_;

	Vector3 betaCompleteLeft_;
	Vector3 betaCompleteRight_;

	acb_t aLeft_; //!< 1/2 + i g/(2vr)
	acb_t aRight_; //!< 1/2 - i g/(2vr)
	acb_t b_; //!< 1/2

	acb_t query_; //!< z used in beta functions
	acb_t queryRes_; //!< result of beta functions

	std::mutex queryMutex_;

	double factor_; //!< v/(2r)
	double velocity_;

	Vector3 centerOrbitLeft_;
	Vector3 centerOrbitRight_;
	double radiusOrbit_;

	VehicleOneFrame frameLeft_;
	VehicleOneFrame frameRight_;
};

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODEL_H_ */
