//
// Created by seedship on 1/22/21.
//

#include "uavAP/Core/Frames/LocalFrame.h"
#include "uavAP/Core/SensorData.h"



LocalFrame::LocalFrame() : yaw(0)
{}

LocalFrame::LocalFrame(const Vector3& origin, FloatingType yaw) : origin(origin), yaw(yaw)
{}

void
LocalFrame::toLocalFrame(SensorData &sd){
	if(!sd.isInLocalFrame) {
		sd.position -= origin;
		sd.attitude[2] -= yaw;
		sd.courseAngle -= yaw;
		sd.isInLocalFrame = true;
	}
}


void
LocalFrame::toGlobalFrame(SensorData &sd) {
	if(sd.isInLocalFrame) {
		sd.position += origin;
		sd.attitude[2] += yaw;
		sd.courseAngle += yaw;
		sd.isInLocalFrame = false;
	}
}
