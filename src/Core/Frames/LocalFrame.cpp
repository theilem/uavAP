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
LocalFrame::toLocalFrame(SensorData &sd) const{
	//NOTE process is the same for ENU and NED
	if(!sd.isInLocalFrame) {
		sd.position = globalPositionToLocal(sd.position);
		sd.attitude[2] -= yaw;
		sd.courseAngle -= yaw;
		sd.isInLocalFrame = true;
	}
}

void
LocalFrame::toGlobalFrame(SensorData &sd) const {
	//NOTE process is the same for ENU and NED
	if(sd.isInLocalFrame) {
		sd.position = localPositionToGlobal(sd.position);
		sd.attitude[2] += yaw;
		sd.courseAngle += yaw;
		sd.isInLocalFrame = false;
	}
}

Vector3
LocalFrame::globalPositionToLocal(const Vector3 &globalPos) const {
	return Eigen::AngleAxisd(-yaw, Vector3::UnitZ()) * (globalPos - origin);
}

Vector3
LocalFrame::localPositionToGlobal(const Vector3 &localPos) const {
	return (Eigen::AngleAxisd(yaw, Vector3::UnitZ()) * localPos) + origin;
}