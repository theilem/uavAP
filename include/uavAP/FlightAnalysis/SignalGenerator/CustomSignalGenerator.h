//
// Created by mirco on 19.03.21.
//

#ifndef UAVAP_CUSTOMSIGNALGENERATOR_H
#define UAVAP_CUSTOMSIGNALGENERATOR_H
#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"

struct CustomSignalGeneratorParams
{
	Parameter<std::vector<unsigned>> timepoints = {{}, "timepoints", true};
	Parameter<std::vector<FloatingType>> values = {{}, "values", true};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & timepoints;
		c & values;
	}

};

class CustomSignalGenerator
		: public ISignalGenerator,
		  public AggregatableObject<ITimeProvider>,
		  public ConfigurableObject<CustomSignalGeneratorParams>
{
public:

	static constexpr auto typeId = "custom";

	void
	initialize() override;

	FloatingType
	getValue() override;

private:

	TimePoint init_;

};

#endif //UAVAP_CUSTOMSIGNALGENERATOR_H
