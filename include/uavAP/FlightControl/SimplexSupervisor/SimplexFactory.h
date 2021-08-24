//
// Created by seedship on 8/24/21.
//

#ifndef UAVAP_SIMPLEXFACTORY_H
#define UAVAP_SIMPLEXFACTORY_H

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/FlightControl/SimplexSupervisor/ISimplexSupervisor.h"
#include "uavAP/FlightControl/SimplexSupervisor/FullStateSimplex.h"

using SimplexFactory = StaticFactory<ISimplexSupervisor, false,
		FullStateSimplex>;


#endif //UAVAP_SIMPLEXFACTORY_H
