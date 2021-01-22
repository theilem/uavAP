//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_NED_H
#define UAVAP_NED_H

#include <uavAP/Core/SensorData.h>
#include "IOrientation.h"

class NED// : public IOrientation
{
public:
	static void
	convert(SensorData& sd);
};


#endif //UAVAP_NED_H
