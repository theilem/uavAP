//
// Created by seedship on 2/2/21.
//

#include "uavAP/FlightControl/Controller/ControlElements/VectoredControlElements.h"

namespace Control
{
template<int N>
VectoredConstant<N>::VectoredConstant(Vector<N> val) :
		val_(val)
{
}

template<int N>
Vector<N>
VectoredConstant<N>::getValue() const
{
	// NOTE : No isnan check, because I couldn't find a vector isnan operation
	// maybe we should add a for loop component wise check
	return val_;
}

template<int N>
InputV<N>::InputV(const Eigen::Matrix<double, N, 1>* in):in_(in)
{}

template<int N>
Eigen::Matrix<double, N, 1>
InputV<N>::getValue() const
{
	return *in_;
}
}
