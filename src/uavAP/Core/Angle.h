/*
 * Angle.h
 *
 *  Created on: Oct 11, 2019
 *      Author: mirco
 */

#ifndef UAVAP_ANGLE_H_
#define UAVAP_ANGLE_H_
#include <cmath>
#include <type_traits>

template<typename Type>
class Angle
{
public:

	static constexpr Type degToRadFactor = M_PI / 180.0;
	static constexpr Type radToDegFactor = 180.0 / M_PI;

	using ValueType = Type;

	Angle() :
			rad_(0)
	{
	}

	explicit
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

	operator Type() const
	{
		return rad_;
	}

	Type
	degrees() const
	{
		return rad_ * radToDegFactor;
	}

private:

	Type rad_;
};

template<typename T>
struct is_angle: public std::false_type
{
};

template<typename T>
struct is_angle<Angle<T>> : public std::true_type
{
};

#endif /* UAVAP_ANGLE_H_ */
