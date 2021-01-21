//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_NED_H
#define UAVAP_NED_H

#include <uavAP/Core/SensorData.h>
#include "IOrientation.h"

class NED : public IOrientation
{
public:
	void
	convert(SensorData& sd) override;
};


#endif //UAVAP_NED_H
