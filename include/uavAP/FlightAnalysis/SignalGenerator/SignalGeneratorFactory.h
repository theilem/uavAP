//
// Created by mirco on 09.03.21.
//

#ifndef UAVAP_SIGNALGENERATORFACTORY_H
#define UAVAP_SIGNALGENERATORFACTORY_H

#include <cpsCore/Framework/StaticFactory.h>

#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"
#include "uavAP/FlightAnalysis/SignalGenerator/SineSignalGenerator.h"

using SignalGeneratorFactory = StaticFactory<ISignalGenerator, false, SineSignalGenerator>;

#endif //UAVAP_SIGNALGENERATORFACTORY_H
