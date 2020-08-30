//
// Created by mirco on 28.08.20.
//

#ifndef UAVAP_LINEARSENSORMANAGER_H
#define UAVAP_LINEARSENSORMANAGER_H

#include <cpsCore/cps_object>
#include "uavAP/API/ap_ext/LinearSensorManagerParams.h"

class LinearSensor : public ConfigurableObject<LinearSensorParams>
{
public:

	template<typename Type, unsigned NumChannels>
	FloatingType
	getValue(const Type (& channels)[NumChannels]) const
	{
		if (params.channel() >= NumChannels)
		{
			CPSLOG_ERROR << "Channel out of range";
			return -1.;
		}
		return static_cast<FloatingType>(channels[params.channel()]) * params.slope() + params.offset();
	}
};


class LinearSensorManager
		: public AggregatableObject<>, public ConfigurableObject<LinearSensorManagerParams>, public IRunnableObject
{

public:

	static constexpr TypeId typeId = "linear_sensor_manager";

	bool
	run(RunStage stage) override;

	template<typename Type, unsigned NumChannels>
	std::map<std::string, FloatingType>
	getValues(const Type (& channels)[NumChannels]) const;

private:

	std::vector<std::pair<std::string, LinearSensor>> sensors_;

};


template<typename Type, unsigned int NumChannels>
std::map<std::string, FloatingType>
LinearSensorManager::getValues(const Type (& channels)[NumChannels]) const
{
	auto values = std::map<std::string, FloatingType>();
	for (const auto&[sensorName, sensor]: sensors_)
	{
		values.emplace(std::make_pair(sensorName, sensor.getValue(channels)));
	}
	return values;
}


#endif //UAVAP_LINEARSENSORMANAGER_H
