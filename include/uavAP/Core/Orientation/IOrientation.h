//
// Created by seedship on 1/19/21.
//

#ifndef UAVAP_IORIENTATION_H
#define UAVAP_IORIENTATION_H

#include <cpsCore/Utilities/EnumMap.hpp>
#include <cpsCore/Utilities/LinearAlgebra.h>
class SensorData;

enum class Orientation
{
	ENU, NED
};

ENUMMAP_INIT(Orientation,
			 {
				 { Orientation::ENU, "enu" },
				 { Orientation::NED, "ned" }
			 }
);

class IOrientation
{
public:

	/**
	 * Converts sensor data into class frame
	 * @param sd
	 */
	virtual void
	convert(SensorData &sd) = 0;
};

#endif //UAVAP_IORIENTATION_H
