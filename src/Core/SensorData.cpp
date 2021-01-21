/*
 * SensorData.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/IFrame.h>
#include "uavAP/Core/SensorData.h"

SensorData&
changeFrame(const IFrame& orig, const IFrame& dest, SensorData& data)
{
	data.position = dest.fromFramePosition(orig, data.position);
	data.velocity = dest.fromFrameDirection(orig, data.velocity);
	data.velocity.frame = dest.getId();
	data.attitude = dest.fromFrameRotation(orig, data.attitude);
	data.courseAngle = dest.fromFrameCourse(orig, data.courseAngle);
	return data; // TODO: Should acceleration be changed?
}
