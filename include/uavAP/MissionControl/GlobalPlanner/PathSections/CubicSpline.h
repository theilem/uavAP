
#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_
#include <uavAP/Core/SensorData.h>

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

struct CubicSpline: public IPathSection
{

	CubicSpline() :
			closestU(0), velocity(0)
	{
	}

	CubicSpline(const Vector3& c0, const Vector3& c1, const Vector3& c2, const Vector3& c3,
			FloatingType velocity) :
			closestU(0), c0(c0), c1(c1), c2(c2), c3(c3), velocity(velocity)
	{
	}

	void
	updateSensorData(const SensorData& data) override
	{
		currentPosition = data.position;

		int maxIter = 10;
		FloatingType convThreshold = 0.0001;

		FloatingType distClosest = ((c0 + c1 * closestU + c2 * pow(closestU, 2)
				+ c3 * pow(closestU, 3)) - currentPosition).squaredNorm();
		FloatingType distZero = (c0 - currentPosition).squaredNorm();
		//Newton method to find closest point
		FloatingType u;
		if (distClosest < distZero)
			u = closestU;
		else
			u = 0;

		for (int i = 0; i < maxIter; ++i)
		{
			Vector3 p = (c0 + c1 * u + c2 * pow(u, 2) + c3 * pow(u, 3)) - currentPosition;
			Vector3 p_prime = c1 + 2 * c2 * u + 3 * c3 * pow(u, 2);
			Vector3 p_2prime = 2 * c2 + 6 * c3 * u;

			FloatingType grad = (p.dot(p_prime)) / (p_prime.dot(p_prime) + p.dot(p_2prime));

			if (std::isnan(grad))
				break;

			u = u - grad;
			if (fabs(grad) < convThreshold)
				break;
		}

		closestU = std::clamp(u, static_cast<FloatingType>(0), static_cast<FloatingType>(1));
	}

	bool
	inTransition() const override
	{
		if (closestU >= 1) //In transition if u for the closest point out of definition for current spline
		{
			return true;
		}
		return false;
	}

	Vector3
	getPositionDeviation() const override
	{
		return (c0 + c1 * closestU + c2 * pow(closestU, 2) + c3 * pow(closestU, 3))
				- currentPosition;
	}

	Vector3
	getDirection() const override
	{
		return (c1 + 2 * c2 * closestU + 3 * c3 * pow(closestU, 2)).normalized();
	}

	FloatingType
	getSlope() const override
	{
		return getDirection().z();
	}

	FloatingType
	getCurvature() const override
	{
		Vector3 dp_du = c1 + 2 * c2 * closestU + 3 * c3 * pow(closestU, 2);
		Vector3 dp_ddu = 2 * c2 + 6 * c3 * closestU;

		FloatingType dPsi_du = (dp_ddu[1] * dp_du[0] - dp_ddu[0] * dp_du[1])
				/ (pow(dp_du[0], 2) + pow(dp_du[1], 2));
		FloatingType du_dt = 1.0 / dp_du.norm();

		return dPsi_du * du_dt; //Clockwise is positive for heading (not counter clockwise)
	}

	std::optional<Vector3>
	getEndPoint() const override
	{
		//u = 1 -> c0 + c1 + c2 + c3
		return c0 + c1 + c2 + c3;
	}

	std::optional<Vector3>
	getEndDirection() const override
	{
		return (c1 + 2 * c2 + 3 * c3).normalized(); //Derivative at u = 1
	}

	std::optional<Vector3>
	getStartingPoint() const override
	{
		return c0; //u = 0 -> c0
	}

	std::optional<Vector3>
	getStartingDirection() const override
	{
		return c1.normalized(); //Derivative at u = 0
	}

	FloatingType
	getVelocity() const override
	{
		return velocity;
	}

	std::string
	getDescription(bool currentState) const override
	{
		std::stringstream ss;
		ss << "CubicSpline: c0: " << c0.transpose() << ", c1: " << c1.transpose()
		   << ", c2: " << c2.transpose() << ", c3: " << c3.transpose() << ", velocity: " << velocity;
		if (currentState)
		   ss << ", closestU: " << closestU;
		return ss.str();
	}

	FloatingType closestU;
	Vector3 currentPosition;

	//Cubic Spline: p(u) = c0 + c1 * u + c2 * u^2 + c3 * u^3
	Vector3 c0;
	Vector3 c1;
	Vector3 c2;
	Vector3 c3;

	FloatingType velocity;

};

namespace dp
{
template<class Archive, typename >
void
serialize(Archive& ar, CubicSpline& t)
{
	ar & t.c0;
	ar & t.c1;
	ar & t.c2;
	ar & t.c3;
	ar & t.closestU;
	ar & t.velocity;
}
}

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_CUBICSPLINE_H_ */
