//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_ENU_H
#define UAVAP_ENU_H

#include <uavAP/Core/SensorData.h>
#include "IOrientation.h"

class ENU : public IOrientation
{
public:
	void
	convert(SensorData& sd) override;
};

#endif //UAVAP_ENU_H
