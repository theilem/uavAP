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
/**
 * @file CustomSerialization.h
 * @brief Defines the custom serialization of objects.
 *
 * New data that should be serialized has to be defined in here, if it cannot be serialized
 * using the generic serialzation methods defined in BasicSerialization.h. Implementation format
 * is:
 * \n
 * \n
 * 	template<class Archive, typename Type>\n
 *	void\n
 *	serialize(Archive& ar, <OBJECT>& t)\n
 *	{\n
 *		ar & t.<PUBLIC_MEMBER1>;\n
 *		ar & t.<PUBLIC_MEMBER2>;\n
 *		...\n
 *	}\n
 * \n
 *
 * This information is used by the serialization and deserialization. Therefore, the order in which the members
 * are arranged in the serialized string will be consistent. If the serialization of <OBJECT>.<PUBLIC_MEMBER> is
 * not defined yet, define it above the new serialize function.
 *
 * @date Aug 28, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_CUSTOMSERIALIZATION_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_CUSTOMSERIALIZATION_H_
#include <boost/variant/variant.hpp>
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Line.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/LocalPlannerParams.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include "uavAP/MissionControl/MissionPlanner/ControlOverride.h"

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

namespace dp
{

template<class Archive, typename Type>
void
serialize(Archive& ar, Vector3& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Vector2& t)
{
	ar & t[0];
	ar & t[1];
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Eigen::Vector3f& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
void
store(Archive& ar, const TimePoint& t)
{
	auto special = t.is_special();
	ar << special;
	if (!special)
	{
		ar << t.date().day_count().as_number();
		ar << t.time_of_day().total_microseconds();
	}
}

template<class Archive, typename Type>
void
load(Archive& ar, TimePoint& t)
{
	bool special;
	ar >> special;
	if (special)
	{
		t = boost::posix_time::not_a_date_time;
		return;
	}
	Date::date_rep_type::int_type days;
	ar >> days;
	TimePoint::time_duration_type::tick_type micros;
	ar >> micros;

	t = TimePoint(Date(days), Microseconds(micros));
}

template<class Archive, typename Type>
void
serialize(Archive& ar, TimePoint& t)
{
	split(ar, t);
}

template<class Archive, typename Type>
void
serialize(Archive& ar, SensorData& t)
{
	ar & t.position;
	ar & t.velocity;
	ar & t.acceleration;
	ar & t.attitude;
	ar & t.angularRate;
	ar & t.angularAcc;
	ar & t.timestamp;
	ar & t.velocityAir;
	ar & t.velocityGround;
	ar & t.propulsionPower;
	ar & t.consumedEnergy;
	ar & t.hasGPSFix;
	ar & t.autopilotActive;
	ar & t.sequenceNr;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, SensorDataLight& t)
{
	ar & t.position;
	ar & t.velocity;
	ar & t.acceleration;
	ar & t.attitude;
	ar & t.angularRate;
	ar & t.timestamp;
	ar & t.velocityAir;
	ar & t.velocityGround;
	ar & t.propulsionPower;
	ar & t.consumedEnergy;
	ar & t.flags;
	ar & t.sequenceNr;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, ControllerTarget& t)
{
	ar & t.velocity;
	ar & t.yawRate;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Mission& t)
{
	ar & t.waypoints;
	ar & t.velocity;
	ar & t.infinite;
}

template<class Archive, typename Type>
void
store(Archive& ar, const EigenLine& t)
{
	ar << t.origin();
	ar << t.direction();
}

template<class Archive, typename Type>
void
load(Archive& ar, EigenLine& t)
{
	Vector3 origin;
	Vector3 direction;
	ar >> origin;
	ar >> direction;
	t = EigenLine(origin, direction);
}

template<class Archive, typename Type>
void
serialize(Archive& ar, EigenLine& t)
{
	split(ar, t);
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Line& t)
{
	ar & static_cast<EigenLine&>(t);

	ar & t.endPoint_;
	ar & t.currentPosition_;
	ar & t.velocity_;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Orbit& t)
{
	ar & t.center_;
	ar & t.normal_;
	ar & t.radius_;
	ar & t.radiusVector_;
	ar & t.velocity_;
	ar & t.currentPosition_;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Curve& t)
{
	ar & static_cast<Orbit&>(t);

	ar & t.endPoint_;
	ar & t.endPointDirection_;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, CubicSpline& t)
{
	ar & t.c0_;
	ar & t.c1_;
	ar & t.c2_;
	ar & t.c3_;
	ar & t.closestU_;
	ar & t.velocity_;
}

template<class Archive, typename Type>
void
store(Archive & ar, const std::shared_ptr<IPathSection> & t)
{
	bool notNull = t != nullptr;
	ar << notNull;
	if (notNull)
	{
		if (auto curve = std::dynamic_pointer_cast<Curve>(t))
		{
			ar << PathSectionType::CURVE;
			ar << *curve;
		}
		else if (auto orbit = std::dynamic_pointer_cast<Orbit>(t))
		{
			ar << PathSectionType::ORBIT;
			ar << *orbit;
		}
		else if (auto line = std::dynamic_pointer_cast<Line>(t))
		{
			ar << PathSectionType::LINE;
			ar << *line;
		}
		else if (auto spline = std::dynamic_pointer_cast<CubicSpline>(t))
		{
			ar << PathSectionType::SPLINE;
			ar << *spline;
		}
		else
		{
			ar << PathSectionType::UNNKNOWN;
		}
	}
}

template<class Archive, typename Type>
void
load(Archive & ar, std::shared_ptr<IPathSection> & t)
{
	bool notNull = false;
	ar >> notNull;
	if (notNull)
	{
		PathSectionType type;
		ar >> type;
		if (type == PathSectionType::CURVE)
		{
			auto curve = std::make_shared<Curve>();
			ar >> *curve;
			t = curve;
		}
		else if (type == PathSectionType::ORBIT)
		{
			auto orbit = std::make_shared<Orbit>();
			ar >> *orbit;
			t = orbit;
		}
		else if (type == PathSectionType::LINE)
		{
			auto line = std::make_shared<Line>();
			ar >> *line;
			t = line;
		}
		else if (type == PathSectionType::SPLINE)
		{
			auto spline = std::make_shared<CubicSpline>();
			ar >> *spline;
			t = spline;
		}
		else
		{
			t = nullptr;
		}
	}
	else
	{
		t = nullptr;
	}
}

template<class Archive, typename Type>
void
serialize(Archive& ar, std::shared_ptr<IPathSection>& t)
{
	split(ar, t);
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Trajectory& t)
{
	ar & t.pathSections;
	ar & t.infinite;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, Control::PID::Parameters& t)
{
	ar & t.kp;
	ar & t.ki;
	ar & t.kd;
	ar & t.imax;
	ar & t.ff;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, PIDTuning& t)
{
	ar & t.pid;
	ar & t.params;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, HelicopterLocalPlannerStatus& t)
{
	ar & t.controllerTarget;
	ar & t.currentPathSection;
	ar & t.directionTarget;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, OverrideActivation& t)
{
	ar & t.activate;

	ar & t.overridePitchTarget;
	ar & t.overrideRollTarget;
	ar & t.overrideVelocityTarget;
	ar & t.overrideClimbRateTarget;
	ar & t.overrideYawRateTarget;

	ar & t.overrideRollOutput;
	ar & t.overrideYawOutput;
	ar & t.overridePitchOutput;
	ar & t.overrideThrottleOutput;
	ar & t.overrideFlapOutput;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, OverrideTarget& t)
{

	ar & t.pitchTarget;
	ar & t.rollTarget;
	ar & t.velocityTarget;
	ar & t.climbRateTarget;
	ar & t.yawRateTarget;

	ar & t.rollOutput;
	ar & t.yawOutput;
	ar & t.pitchOutput;
	ar & t.throttleOutput;
	ar & t.flapOutput;
}

template<class Archive, typename Type>
void
serialize(Archive& ar, ControlOverride& t)
{
	ar & t.overrideManeuverPlanner;
	ar & t.target;
	ar & t.activation;
}

}

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_CUSTOMSERIALIZATION_H_ */
