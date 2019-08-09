////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * Spline.h
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

struct CubicSpline: public IPathSection
{

	CubicSpline() :
			closestU_(0), velocity_(0)
	{
	}

	CubicSpline(const Vector3& c0, const Vector3& c1, const Vector3& c2, const Vector3& c3,
			double velocity) :
			closestU_(0), c0_(c0), c1_(c1), c2_(c2), c3_(c3), velocity_(velocity)
	{
	}

	void
	updatePosition(const Vector3& pos) override
	{
		currentPosition_ = pos;

		int maxIter = 10;
		double convThreshold = 0.0001;

		double distClosest = ((c0_ + c1_ * closestU_ + c2_ * pow(closestU_, 2)
				+ c3_ * pow(closestU_, 3)) - currentPosition_).squaredNorm();
		double distZero = (c0_ - currentPosition_).squaredNorm();
		//Newton method to find closest point
		double u;
		if (distClosest < distZero)
			u = closestU_;
		else
			u = 0;

		for (int i = 0; i < maxIter; ++i)
		{
			Vector3 p = (c0_ + c1_ * u + c2_ * pow(u, 2) + c3_ * pow(u, 3)) - currentPosition_;
			Vector3 p_prime = c1_ + 2 * c2_ * u + 3 * c3_ * pow(u, 2);
			Vector3 p_2prime = 2 * c2_ + 6 * c3_ * u;

			double grad = (p.dot(p_prime)) / (p_prime.dot(p_prime) + p.dot(p_2prime));

			if (std::isnan(grad))
				break;

			u = u - grad;
			if (fabs(grad) < convThreshold)
				break;
		}

		if (u < 0)
			u = 0;
		closestU_ = u;
	}

	bool
	inTransition() const override
	{
		//Hack to set closestU_ to 0
		if (closestU_ >= 1) //In transition if u for the closest point out of definition for current spline
		{
//			CubicSpline* s = const_cast<CubicSpline*>(this);
//			s->closestU_ = 0;
			return true;
		}
		return false;
	}

	Vector3
	getPositionDeviation() const override
	{
		return (c0_ + c1_ * closestU_ + c2_ * pow(closestU_, 2) + c3_ * pow(closestU_, 3))
				- currentPosition_;
	}

	Vector3
	getDirection() const override
	{
		return (c1_ + 2 * c2_ * closestU_ + 3 * c3_ * pow(closestU_, 2)).normalized();
	}

	double
	getSlope() const override
	{
		return getDirection().z();
	}

	double
	getCurvature() const override
	{
		Vector3 dp_du = c1_ + 2 * c2_ * closestU_ + 3 * c3_ * pow(closestU_, 2);
		Vector3 dp_ddu = 2 * c2_ + 6 * c3_ * closestU_;

		double dPsi_du = (dp_ddu[1] * dp_du[0] - dp_ddu[0] * dp_du[1])
				/ (pow(dp_du[0], 2) + pow(dp_du[1], 2));
		double du_dt = 1.0 / dp_du.norm();

		return dPsi_du * du_dt; //Clockwise is positive for heading (not counter clockwise)
	}

	Vector3
	getEndPoint() const override
	{
		//u = 1 -> c0 + c1 + c2 + c3
		return c0_ + c1_ + c2_ + c3_;
	}

	double
	getVelocity() const override
	{
		return velocity_;
	}

	double closestU_;
	Vector3 currentPosition_;

	//Cubic Spline: p(u) = c0 + c1 * u + c2 * u^2 + c3 * u^3
	Vector3 c0_;
	Vector3 c1_;
	Vector3 c2_;
	Vector3 c3_;

	double velocity_;

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, CubicSpline& t)
{
	ar & t.c0_;
	ar & t.c1_;
	ar & t.c2_;
	ar & t.c3_;
	ar & t.closestU_;
	ar & t.velocity_;
}
}

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_ */
