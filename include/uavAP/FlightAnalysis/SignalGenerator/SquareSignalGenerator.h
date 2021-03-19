//
// Created by mirco on 19.03.21.
//

#ifndef UAVAP_SQUARESIGNALGENERATOR_H
#define UAVAP_SQUARESIGNALGENERATOR_H

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/PeriodicSignalParams.h"

class SquareSignalGenerator
		: public ISignalGenerator,
		  public AggregatableObject<ITimeProvider>,
		  public ConfigurableObject<PeriodicSignalParams>
{
public:

	static constexpr auto typeId = "square";

	void
	initialize() override;

	FloatingType
	getValue() override;

private:

	TimePoint init_;

};



#endif //UAVAP_SQUARESIGNALGENERATOR_H
