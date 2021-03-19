//
// Created by mirco on 09.03.21.
//

#ifndef UAVAP_SIGNALGENERATORFACTORY_H
#define UAVAP_SIGNALGENERATORFACTORY_H

#include <cpsCore/Framework/StaticFactory.h>

#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/SineSignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/SquareSignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/CustomSignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/MultiSineSignalGenerator.h"

using SignalGeneratorFactory = StaticFactory<ISignalGenerator, false,
		SineSignalGenerator,
		SquareSignalGenerator,
		CustomSignalGenerator,
		MultiSineSignalGenerator
>;

#endif //UAVAP_SIGNALGENERATORFACTORY_H
