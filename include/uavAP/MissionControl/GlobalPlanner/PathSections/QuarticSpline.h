/*
 * Spline.h
 *
 *  Created on: Dec 12, 2024
 *  Author: Mirco Theile (mirco.theile@tum.de)
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_

#include <uavAP/Core/SensorData.h>

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

struct QuarticSpline : public IPathSection
{
	QuarticSpline() = default;

	QuarticSpline(const QuarticSpline& other) = default;

	QuarticSpline(const Vector3& c0, const Vector3& c1, const Vector3& c2, const Vector3& c3, const Vector3& c4,
				  FloatingType velocity) :
			closestU_(0), c0_(c0), c1_(c1), c2_(c2), c3_(c3), c4_(c4), velocity_(velocity)
	{
	}

	Vector3
	positionFromU(FloatingType u) const
	{
		return c0_ + c1_ * u + c2_ * pow(u, 2) + c3_ * pow(u, 3) + c4_ * pow(u, 4);
	}

	void
	updateSensorData(const SensorData& data) override
	{
		currentPosition_ = data.position;

		int maxIter = 10;
		FloatingType convThreshold = 0.0001;
		auto posClosest = positionFromU(closestU_);

		FloatingType distClosest = (posClosest - currentPosition_).squaredNorm();
		FloatingType distZero = (c0_ - currentPosition_).squaredNorm();
		//Newton method to find closest point
		FloatingType u;
		if (distClosest < distZero)
			u = closestU_;
		else
			u = 0;

		for (int i = 0; i < maxIter; ++i)
		{
			Vector3 p = positionFromU(u) - currentPosition_;
			Vector3 p_prime = c1_ + 2 * c2_ * u + 3 * c3_ * pow(u, 2) + 4 * c4_ * pow(u, 3);
			Vector3 p_2prime = 2 * c2_ + 6 * c3_ * u + 12 * c4_ * pow(u, 2);

			FloatingType grad = (p.dot(p_prime)) / (p_prime.dot(p_prime) + p.dot(p_2prime));

			if (std::isnan(grad))
				break;

			u = u - grad;
			if (fabs(grad) < convThreshold)
				break;
		}

		if (u < 0)
			u = 0;
		if (u > 1)
			u = 1;
		closestU_ = u;
	}

	bool
	inTransition() const override
	{
		if (closestU_ >= 1) //In transition if u for the closest point out of definition for current spline
		{
			return true;
		}
		return false;
	}

	Vector3
	getPositionDeviation() const override
	{
		return positionFromU(closestU_) - currentPosition_;
	}

	Vector3
	getDirection() const override
	{
		return (c1_ + 2 * c2_ * closestU_ + 3 * c3_ * pow(closestU_, 2) + 4 * c4_ * pow(closestU_, 3)).normalized();
	}

	FloatingType
	getSlope() const override
	{
		return getDirection().z();
	}

	FloatingType
	getCurvature() const override
	{
		Vector3 dp_du = c1_ + 2 * c2_ * closestU_ + 3 * c3_ * pow(closestU_, 2) + 4 * c4_ * pow(closestU_, 3);
		Vector3 dp_ddu = 2 * c2_ + 6 * c3_ * closestU_ + 12 * c4_ * pow(closestU_, 2);

		FloatingType dPsi_du = (dp_ddu[1] * dp_du[0] - dp_ddu[0] * dp_du[1])
							   / pow((pow(dp_du[0], 2) + pow(dp_du[1], 2)), 3.0 / 2.0);

		return dPsi_du;
	}

	Vector3
	getEndPoint() const override
	{
		return c0_ + c1_ + c2_ + c3_ + c4_;
	}

	FloatingType
	getVelocity() const override
	{
		return velocity_;
	}

	FloatingType closestU_{0.0};
	Vector3 currentPosition_;

	//Quartic Spline: p(u) = c0 + c1 * u + c2 * u^2 + c3 * u^3 + c4 * u^4
	Vector3 c0_;
	Vector3 c1_;
	Vector3 c2_;
	Vector3 c3_;
	Vector3 c4_;

	FloatingType velocity_{};

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, QuarticSpline& t)
{
	ar & t.c0_;
	ar & t.c1_;
	ar & t.c2_;
	ar & t.c3_;
	ar & t.c4_;
	ar & t.closestU_;
	ar & t.velocity_;
}
}

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_ */
