//
// Created by seedship on 8/24/21.
//

#ifndef UAVAP_FULLSTATESIMPLEX_H
#define UAVAP_FULLSTATESIMPLEX_H

#include <cpsCore/cps_object>
#include "uavAP/FlightControl/SimplexSupervisor/ISimplexSupervisor.h"

class IActuationIO;

class FullStateSimplex: public AggregatableObject<IActuationIO>,
						public ISimplexSupervisor
{
public:
	static constexpr TypeId typeId = "full_state_simplex";

//	FullStateSimplex();

	void
	setControllerOutput(const ControllerOutput& out) override;
};


#endif //UAVAP_FULLSTATESIMPLEX_H
