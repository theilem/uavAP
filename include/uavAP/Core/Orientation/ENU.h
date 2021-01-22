//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_ENU_H
#define UAVAP_ENU_H

#include <uavAP/Core/SensorData.h>
#include "IOrientation.h"

class ENU// : public IOrientation
{
public:
	static void
	convert(SensorData& sd);
};

#endif //UAVAP_ENU_H
