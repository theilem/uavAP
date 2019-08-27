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
 * SensingActuationIO.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_IO_SENSINGACTUATIONIO_ISENSINGACTUATIONIO_H_
#define UAVAP_FLIGHTCONTROL_IO_SENSINGACTUATIONIO_ISENSINGACTUATIONIO_H_
#include "uavAP/Core/SensorData.h"

#ifndef ERIKA
#include <boost/signals2/signal.hpp>
#endif


struct ControllerOutput;

class ISensingActuationIO
{
public:

	static constexpr const char* const typeId = "sens_act_io";

	virtual
	~ISensingActuationIO() = default;

	virtual void
	setControllerOutput(const ControllerOutput& out) = 0;

	virtual SensorData
	getSensorData() const = 0;

#ifdef ERIKA
	using OnSensorDataSlot = std::function<void(const SensorData&)>;
	using OnSensorDataConnection = int;
#else
	using OnSensorData = boost::signals2::signal<void(const SensorData&)>;
	using OnSensorDataSlot = boost::signals2::signal<void(const SensorData&)>::slot_type;
	using OnSensorDataConnection = boost::signals2::connection;
#endif
	virtual OnSensorDataConnection
	subscribeOnSensorData(const OnSensorDataSlot& slot) = 0;
};

#endif /* UAVAP_FLIGHTCONTROL_IO_SENSINGACTUATIONIO_ISENSINGACTUATIONIO_H_ */
