/*
 * Trajectory.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_
#define UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_


#include <vector>
#include <memory>

#include <cpsCore/Utilities/DataPresentation/detail/Split.h>

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Line.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/QuarticSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Spiral.h"


using PathSections = std::vector<std::shared_ptr<IPathSection>>;
using PathSectionIterator = std::vector<std::shared_ptr<IPathSection>>::iterator;

inline std::ostream&
operator<<(std::ostream& os, const PathSections& p)
{
    os << "[" << std::endl;
    for (const auto& section : p)
    {
        if (section)
        {
            os << section->getDescription(false) << ", ";
        }
        else
        {
            os << "nullptr, ";
        }
        os << std::endl;
    }
    os << "]";
    return os;
}

struct Trajectory
{
    PathSections aperiodicPart;
    PathSections periodicPart;
    Trajectory() = default;

    Trajectory(const PathSections& sections, bool inf):
        aperiodicPart(inf ? PathSections() : sections),
        periodicPart(inf ? sections : PathSections())
    {
    }

    Trajectory(PathSections aperiodic, PathSections periodic):
        aperiodicPart(std::move(aperiodic)),
        periodicPart(std::move(periodic))
    {
    }
};


inline std::ostream&
operator<<(std::ostream& os, const Trajectory& traj)
{
    os << "Aperiodic part: " << std::endl;
    os << traj.aperiodicPart << std::endl;
    os << "Periodic part: " << std::endl;
    os << traj.periodicPart << std::endl;
    return os;
}


namespace dp
{
    template <class Archive, typename>
    void
    store(Archive& ar, const std::shared_ptr<IPathSection>& t)
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
            else if (auto spiral = std::dynamic_pointer_cast<Spiral>(t))
            {
                ar << PathSectionType::SPIRAL; // Spiral is a special case of Orbit
                ar << *spiral;
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
            else if (auto quartic = std::dynamic_pointer_cast<QuarticSpline>(t))
            {
                ar << PathSectionType::QUARTIC_SPLINE;
                ar << *quartic;
            }
            else
            {
                ar << PathSectionType::UNKNOWN;
            }
        }
    }

    template <class Archive, typename>
    void
    load(Archive& ar, std::shared_ptr<IPathSection>& t)
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
            else if (type == PathSectionType::SPIRAL)
            {
                auto spiral = std::make_shared<Spiral>();
                ar >> *spiral;
                t = spiral;
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
            else if (type == PathSectionType::QUARTIC_SPLINE)
            {
                auto quartic = std::make_shared<QuarticSpline>();
                ar >> *quartic;
                t = quartic;
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

    template <class Archive, typename>
    void
    serialize(Archive& ar, std::shared_ptr<IPathSection>& t)
    {
        split(ar, t);
    }

    template <class Archive, typename>
    void
    serialize(Archive& ar, Trajectory& t)
    {
        ar & t.aperiodicPart;
        ar & t.periodicPart;
    }
} //namespace dp

#endif /* UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_ */
