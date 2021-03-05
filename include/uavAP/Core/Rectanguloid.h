/*
 * Rectanguloid.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Configuration/Parameter.hpp>

struct Rectanguloid
{
	Parameter<Vector3> center = {{}, "center", true};
	Parameter<FloatingType> majorSideLength = {{}, "major_side_length", true};
	Parameter<FloatingType> minorSideLength = {{}, "minor_side_length", true};
	Parameter<FloatingType> majorSideOrientation = {{}, "major_side_orientation", true};
	Parameter<FloatingType> height = {{}, "height", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & center;
		c & majorSideLength;
		c & minorSideLength;
		c & majorSideOrientation;
		c & height;
	}

	bool
	isInside(const Vector3& point) const
	{
		Vector3 diff = point - center();
		if (std::fabs(diff[2]) > height() / 2.0)
			return false;

		Vector2 offset = diff.head(2);
		Eigen::Rotation2D rot(-majorSideOrientation());
		offset = rot * offset;

		if (std::fabs(offset[0]) > majorSideLength() / 2.0 || std::fabs(offset[1]) > minorSideLength() / 2.0)
			return false;
		return true;
	}
};



#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_ */
