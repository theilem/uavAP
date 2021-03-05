//
// Created by mirco on 26.02.21.
//

#ifndef UAVAP_ISENSINGIO_H
#define UAVAP_ISENSINGIO_H

#include <boost/signals2.hpp>
#include "uavAP/Core/SensorData.h"

class ISensingIO
{
public:
	static constexpr const char* const typeId = "sens_act_io";

	virtual
	~ISensingIO() = default;

	virtual SensorData
	getSensorData() const = 0;

	using OnSensorData = boost::signals2::signal<void(const SensorData&)>;
	using OnSensorDataSlot = boost::signals2::signal<void(const SensorData&)>::slot_type;
	using OnSensorDataConnection = boost::signals2::connection;

	virtual OnSensorDataConnection
	subscribeOnSensorData(const OnSensorDataSlot& slot) = 0;
};

#endif //UAVAP_ISENSINGIO_H
