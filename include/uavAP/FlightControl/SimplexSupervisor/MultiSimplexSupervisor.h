//
// Created by seedship on 8/24/21.
//

#ifndef UAVAP_MULTISIMPLEXSUPERVISOR_H
#define UAVAP_MULTISIMPLEXSUPERVISOR_H

#include <cpsCore/cps_object>
#include "uavAP/FlightControl/SimplexSupervisor/ISimplexSupervisor.h"

class IActuationIO;
class ISensingIO;

class MultiSimplexSupervisor: public AggregatableObject<IActuationIO, ISensingIO>,
							  public ISimplexSupervisor
{
public:
	static constexpr TypeId typeId = "full_state_simplex";

//	FullStateSimplex();

	void
	setControllerOutput(const ControllerOutput& out) override;

private:
	VectorN<9>
	calculateRawState() const;
};


#endif //UAVAP_MULTISIMPLEXSUPERVISOR_H
