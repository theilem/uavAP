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
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Frames/VehicleOneFrame.h"
#include "uavAP/Core/Object/AggregatableObject.hpp"
#include "uavAP/MissionControl/WindAnalysis/WindAnalysisStatus.h"
#include "uavAP/MissionControl/Geofencing/IGeofencingModel.h"
#include "uavAP/Core/PropertyMapper/ConfigurableObject.hpp"
#include "uavAP/MissionControl/Geofencing/ConstRollRateModelParams.h"

class ConstRollRateModel: public IGeofencingModel,
		public AggregatableObject<>,
		public ConfigurableObject<ConstRollRateModelParams>
{
public:

	static constexpr const char* const typeId = "roll_rate";

	ConstRollRateModel();

	~ConstRollRateModel();

	bool
	updateModel(const SensorData& data, const WindInfo& wind) override;

	std::vector<Vector3>
	getCriticalPoints(const Edge& edge, RollDirection dir) override;

	/**
	 * @brief If frameleft/right is set it is calculating the yaw given a specified roll angle
	 * @param roll
	 * @param dir
	 * @return
	 */
	FloatingType
	calculateYaw(FloatingType roll, RollDirection dir);

	std::vector<FloatingType>
	calculateRoll(FloatingType yaw, RollDirection dir);

	Vector3
	toVector(const acb_t& complex);

private:

	std::vector<Vector3>
	getCriticalPointsOrbit(FloatingType course, FloatingType end, RollDirection dir);

	FloatingType
	getTimeFromYawOrbit(FloatingType yaw, RollDirection dir);

	Vector3
	getPositionOrbit(FloatingType time, RollDirection dir);

	Vector3
	getPositionCurve(FloatingType time, RollDirection dir);

	/**
	 * @brief If frameleft/right is set it is calculating the position given a specified roll angle
	 * @param roll
	 * @param dir
	 * @return
	 */
	Vector3
	calculatePoint(FloatingType roll, RollDirection dir);

	Vector3
	calculatePointOrbit(FloatingType time, RollDirection dir);

	Vector3
	getWindDisplacement(FloatingType time);

	FloatingType
	calculateYaw(FloatingType course);

	FloatingType
	calculateCourse(FloatingType yaw);

	Control::LowPassFilter airspeedFilter_;

	Vector3 wind_;

	FloatingType currentRoll_;

	Vector3 betaCompleteLeft_;
	Vector3 betaCompleteRight_;

	acb_t aLeft_; //!< 1/2 + i g/(2vr)
	acb_t aRight_; //!< 1/2 - i g/(2vr)
	acb_t b_; //!< 1/2

	acb_t query_; //!< z used in beta functions
	acb_t queryRes_; //!< result of beta functions

	Mutex queryMutex_;

	FloatingType factor_; //!< v/(2r)
	FloatingType velocity_;

	Vector3 endpointLeft_;
	FloatingType yawEndLeft_;
	Vector3 endpointRight_;
	FloatingType yawEndRight_;
	FloatingType yawrateOrbit_;

	VehicleOneFrame frameLeft_;
	VehicleOneFrame frameRight_;
};

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_CONSTROLLRATEMODEL_H_ */
