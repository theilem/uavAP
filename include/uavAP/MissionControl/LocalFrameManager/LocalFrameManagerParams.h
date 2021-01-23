//
// Created by mirco on 04.01.20.
//

#ifndef UAVAP_LOCALFRAMEMANAGERPARAMS_H
#define UAVAP_LOCALFRAMEMANAGERPARAMS_H

#include "uavAP/Core/Frames/LocalFrame.h"

struct LocalFrameManagerParams
{
	Parameter<Angle<FloatingType>> yaw = {{}, "yaw", true};
	Parameter<Vector3> origin = {Vector3(0,0,0), "origin", true};

	LocalFrame
	toLocalFrame() const
	{
		return LocalFrame(origin(), yaw());
	}

	template <typename Config>
	void
	configure(Config& c)
	{
		c & yaw;
		c & origin;
	}
};


#endif //UAVAP_LOCALFRAMEMANAGERPARAMS_H
