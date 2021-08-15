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
	convert(SensorData& sd, Frame velocityFrame = Frame::INERTIAL, Frame accelerationFrame = Frame::BODY, Frame angularRateFrame = Frame::BODY);

	static void
	setUVWDot(SensorData& sd);
};


#endif //UAVAP_NED_H
