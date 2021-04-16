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
	FramedVector3():Vector3(){}

	template<typename OtherDerived>
	FramedVector3(const Eigen::MatrixBase<OtherDerived>& other) : Vector3(other) {}

	// We need Eigen 3.4 to be able to initialize from initializer lists
	// https://gitlab.com/libeigen/eigen/-/issues/2192
//	FramedVector3(const std::initializer_list<double>& other) : Vector3({other}) {}

	template<typename Derived>
	FramedVector3& operator=(const Eigen::MatrixBase<Derived>& other){
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
	ar & t[0];
	ar & t[1];
	ar & t[2];
	ar & t.frame;
}
}


#endif //UAVAP_FRAMEDVECTOR3_H
