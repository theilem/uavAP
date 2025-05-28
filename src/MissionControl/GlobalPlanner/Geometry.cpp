//
// Created by Mirco Theile on 27/5/25.
//

#include "uavAP/MissionControl/GlobalPlanner/Geometry.h"

std::optional<Tangent>
computeInternalTangentPoint(const Vector2& c1, double r1,
                            const Vector2& c2, double r2,
                            OrbitDirection direction)
{
    Vector2 d = c2 - c1;
    double D = d.norm();
    double r = (r1 + r2) / D;
    if (r > 1.0)
    {
        // The circles are intersecting or one is inside the other
        return std::nullopt;
    }

    Vector2 d_hat = d / D;
    Vector2 n_hat(-d_hat.y(), d_hat.x()); // Perpendicular to d_hat

    double theta = std::acos(r);

    // Two possible tangent directions
    Vector2 e = std::cos(theta) * d_hat;
    if (direction == OrbitDirection::CCW)
        e -= std::sin(theta) * n_hat;
    else
        e += std::sin(theta) * n_hat;

    e = e.normalized(); // Normalize to get unit radial direction

    Vector2 n(-e.y(), e.x()); // Perpendicular to e
    if (direction == OrbitDirection::CW)
    {
        n = -n; // Reverse direction for CW
    }

    Vector2 p1 = c1 + r1 * e; // Tangent point on circle C1
    Vector2 p2 = c2 - r2 * e; // Tangent point on circle C2 which is on the other side of the tangent line
    return {{p1, p2, n}};
}

std::optional<Tangent>
computeExternalTangentPoint(const Vector2& c1, double r1,
                            const Vector2& c2, double r2,
                            OrbitDirection direction)
{
    Vector2 d = c2 - c1;
    double D = d.norm();
    double r = (r1 - r2) / D;
    if (std::abs(r) > 1.0)
    {
        // One circle is inside the other.
        return std::nullopt;
    }
    Vector2 d_hat = d / D;
    Vector2 n_hat(-d_hat.y(), d_hat.x()); // Perpendicular to d_hat

    double theta = std::acos(r);
    Vector2 e = std::cos(theta) * d_hat;
    if (direction == OrbitDirection::CCW)
        e -= std::sin(theta) * n_hat;
    else
        e += std::sin(theta) * n_hat;

    e.normalize(); // Normalize to get unit tangent direction
    Vector2 n(-e.y(), e.x()); // Perpendicular to e
    if (direction == OrbitDirection::CW)
    {
        n = -n; // Reverse direction for CCW
    }
    Vector2 p1 = c1 + r1 * e; // Tangent point on circle C1
    Vector2 p2 = c2 + r2 * e;
    return {{p1, p2, n}};
}
