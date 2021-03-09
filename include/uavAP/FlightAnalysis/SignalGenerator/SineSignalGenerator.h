//
// Created by mirco on 09.03.21.
//

#ifndef UAVAP_SINUSSIGNALGENERATOR_H
#define UAVAP_SINUSSIGNALGENERATOR_H

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"

struct SineSignalGeneratorParams
{
	Parameter<FloatingType> amplitude = {1.0, "amplitude", true};
	Parameter<FloatingType> frequency = {1.0, "frequency", true};
	Parameter<Angle<FloatingType>> phase = {{}, "phase", false};
	Parameter<FloatingType> offset = {0.0, "offset", false};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & amplitude;
		c & frequency;
		c & phase;
		c & offset;
	}
};

class SineSignalGenerator
		: public ISignalGenerator,
		  public AggregatableObject<ITimeProvider>,
		  public ConfigurableObject<SineSignalGeneratorParams>
{
public:

	static constexpr auto typeId = "sine";

	void
	initialize() override;

	FloatingType
	getValue() override;

private:

	TimePoint init_;

};

#endif //UAVAP_SINUSSIGNALGENERATOR_H
