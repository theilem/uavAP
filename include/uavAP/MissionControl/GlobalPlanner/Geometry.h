//
// Created by Mirco Theile on 27/5/25.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <utility>  // for std::pair
#include <stdexcept>

#include "cpsCore/Utilities/LinearAlgebra.h"
#include "PathSections/Orbit.h"

struct Tangent
{
    Vector2 pointA;
    Vector2 pointB;
    Vector2 direction;  // Direction of the tangent line
};

// Returns the internal tangent point with direction on C1 (center c1, radius r1) and the  such that
// the tangent line departs in the given direction (CW or CCW) toward circle C2
std::optional<Tangent>
computeInternalTangentPoint(const Vector2& c1, double r1,
                            const Vector2& c2, double r2,
                            OrbitDirection direction);

std::optional<Tangent>
computeExternalTangentPoint(const Vector2& c1, double r1,
                            const Vector2& c2, double r2,
                            OrbitDirection direction);

#endif //GEOMETRY_H
