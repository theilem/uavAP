//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_FRAMEDVECTOR3_H
#define UAVAP_FRAMEDVECTOR3_H


#include <cpsCore/Utilities/LinearAlgebra.h>
#include "uavAP/Core/Frames/IFrame.h"


class FramedVector3 : public Vector3
{
public:
	// Taken from here: https://eigen.tuxfamily.org/dox/TopicCustomizing_InheritingMatrix.html
	FramedVector3(void):Vector3(){}

	template<typename OtherDerived>
	FramedVector3(const Eigen::MatrixBase<OtherDerived>& other) : Vector3(other) {}

	template<typename OtherDerived>
	FramedVector3& operator=(const Eigen::MatrixBase<OtherDerived>& other){
		this->Vector3::operator=(other);
		return *this;
	}

	Frame frame = Frame::INERTIAL;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, FramedVector3& t)
{
	ar & (Vector3)t;
	ar & t.frame;
}
}

//	Vector3 data = {0., 0., 0.};

//	const FloatingType&
//	x() const
//	{
//		return data.x();
//	}
//
//	const FloatingType&
//	y() const
//	{
//		return data.y();
//	}
//
//	const FloatingType&
//	z() const
//	{
//		return data.z();
//	}
//
//	FloatingType&
//	x()
//	{
//		return data.x();
//	}
//
//	FloatingType&
//	y()
//	{
//		return data.y();
//	}
//
//	FloatingType&
//	z()
//	{
//		return data.z();
//	}


#endif //UAVAP_FRAMEDVECTOR3_H
