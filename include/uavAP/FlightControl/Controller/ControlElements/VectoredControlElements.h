//
// Created by seedship on 2/2/21.
//

#ifndef UAVAP_VECTOREDCONTROLELEMENTS_H
#define UAVAP_VECTOREDCONTROLELEMENTS_H


#include <Eigen/src/Core/Matrix.h>
#include "IControlElement.h"

namespace Control
{
// Work in progress, currently not added to CMake Target

// TODO there might be a way to eliminate this template parameter N, but I dont know how
// https://stackoverflow.com/questions/66014169
template<int N>
using Vector = Eigen::Matrix<FloatingType, N, 1>;

template<int R, int C>
using Matrix = Eigen::Matrix<FloatingType, R, C>;

template<int N>
class VectoredConstant: public IControlElement
{
public:
	VectoredConstant(Vector<N> val);

	Vector<N>
	getValue() const override;

private:

	Vector<N> val_;
};

template<int R, int C>
class MatrixGain : public IControlElement
{
public:
	MatrixGain(Element in, Matrix<R, C> gain);

	/**
	 * Standard matrix multiplication, output = K * in
	 * @return
	 */
	Vector<R>
	getValue() const override;

private:

	Element in_;

	Matrix<R, C> gain_;

};


template<int N>
class InputV : public IControlElement
{
public:
	InputV(const Eigen::Matrix<double, N, 1>* in);

	Eigen::Matrix<double, N, 1> getValue() const override;

private:
	Eigen::Matrix<double, N, 1> const * in_;
};
};


#endif //UAVAP_VECTOREDCONTROLELEMENTS_H
