/*
 * Angle.h
 *
 *  Created on: Oct 11, 2019
 *      Author: mirco
 */

#ifndef UAVAP_ANGLE_H_
#define UAVAP_ANGLE_H_
#include <cmath>

template<typename Type>
class Angle
{
public:

	static constexpr Type degToRadFactor = M_PI / 180.0;
	static constexpr Type radToDegFactor = 180.0 / M_PI;

	Angle(const Type& deg) :
			rad_(deg * degToRadFactor)
	{
	}

	Angle<Type>&
	operator=(const Type& deg)
	{
		rad_ = deg * degToRadFactor;
		return *this;
	}

	const Type&
	operator()() const
	{
		return rad_;
	}

private:

	Type rad_;
};

//template <typename Type>
//inline const Type&
//operator=(const Angle<Type>& angle)
//{
//	return angle();
//}

#endif /* UAVAP_ANGLE_H_ */
