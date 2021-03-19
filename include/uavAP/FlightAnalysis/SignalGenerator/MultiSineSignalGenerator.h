//
// Created by mirco on 19.03.21.
//

#ifndef UAVAP_MULTISINESIGNALGENERATOR_H
#define UAVAP_MULTISINESIGNALGENERATOR_H

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/PeriodicSignalParams.h"

struct MultiSineSignalGeneratorParams
{
	Parameter<std::vector<PeriodicSignalParams>> sines = {{}, "sines", true};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & sines;
	}

};

class MultiSineSignalGenerator
		: public ISignalGenerator,
		  public AggregatableObject<ITimeProvider>,
		  public ConfigurableObject<MultiSineSignalGeneratorParams>
{
public:

	static constexpr auto typeId = "multi_sine";

	void
	initialize() override;

	FloatingType
	getValue() override;

private:

	TimePoint init_;

};

#endif //UAVAP_MULTISINESIGNALGENERATOR_H
