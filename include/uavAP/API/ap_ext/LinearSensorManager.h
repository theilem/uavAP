//
// Created by mirco on 28.08.20.
//

#ifndef UAVAP_LINEARSENSORMANAGER_H
#define UAVAP_LINEARSENSORMANAGER_H

#include <cpsCore/cps_object>
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include "uavAP/API/ap_ext/LinearSensorManagerParams.h"

class LinearSensor : public ConfigurableObject<LinearSensorParams>
{
public:

	void
	initialize()
	{
		if (params.filter())
		{
			filter_ = Control::LowPassFilterGeneric<FloatingType>();
			filter_->setParams(*params.filter());
		}
	}

	template<typename Type, unsigned NumChannels>
	FloatingType
	getValue(const Type (& channels)[NumChannels]) const
	{
		if (params.channel() >= NumChannels)
		{
			CPSLOG_ERROR << "Channel out of range";
			return -1.;
		}
		auto value = static_cast<FloatingType>(channels[params.channel()]) * params.slope() + params.offset();
		if (filter_)
			return filter_->update(value);
		return value;
	}
private:
	mutable Optional<Control::LowPassFilterGeneric<FloatingType>> filter_;
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
