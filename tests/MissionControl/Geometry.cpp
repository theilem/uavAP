//
// Created by Mirco Theile on 27/5/25.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/MissionControl/GlobalPlanner/Geometry.h"

TEST_CASE("Internal Tangent Point Calculation", "[geometry]")
{
    Vector2 c1(0, 0);
    double r1 = 10;
    Vector2 c2(20, -15);
    double r2 = 5;

    auto tangentPoint = computeInternalTangentPoint(c1, r1, c2, r2, OrbitDirection::CCW);
    REQUIRE(tangentPoint.has_value());

    auto [pointA, pointB, direction] = tangentPoint.value();
    CHECK(pointA.x() == Approx(0).margin(0.01));
    CHECK(pointA.y() == Approx(-10).margin(0.01));
    CHECK(pointB.x() == Approx(20).margin(0.01));
    CHECK(pointB.y() == Approx(-10).margin(0.01));
    CHECK(direction.x() == Approx(1).margin(0.01));
    CHECK(direction.y() == Approx(0).margin(0.01));

    Vector2 c3(0, 11);
    double r3 = 2;
    auto tangentPoint2 = computeInternalTangentPoint(c1, r1, c3, r3, OrbitDirection::CCW);
    REQUIRE(!tangentPoint2.has_value());
}

TEST_CASE("External Tangent Point Calculation", "[geometry]")
{
    Vector2 c1(0, 0);
    double r1 = 10;
    Vector2 c2(20, 0);
    double r2 = 10;

    auto tangentPoint = computeExternalTangentPoint(c1, r1, c2, r2, OrbitDirection::CCW);
    REQUIRE(tangentPoint.has_value());

    auto [pointA, pointB, direction] = tangentPoint.value();
    CHECK(pointA.x() == Approx(0).margin(0.01));
    CHECK(pointA.y() == Approx(-10).margin(0.01));
    CHECK(pointB.x() == Approx(20).margin(0.01));
    CHECK(pointB.y() == Approx(-10).margin(0.01));
    CHECK(direction.x() == Approx(1).margin(0.01));
    CHECK(direction.y() == Approx(0).margin(0.01));

    Vector2 c3(0, 5);
    double r3 = 2;
    auto tangentPoint2 = computeExternalTangentPoint(c1, r1, c3, r3, OrbitDirection::CCW);
    REQUIRE(!tangentPoint2.has_value());

    Vector2 c4(20, 0);
    double r4 = 20;
    auto tangentPoint3 = computeExternalTangentPoint(c1, r1, c4, r4, OrbitDirection::CCW);
    REQUIRE(tangentPoint3.has_value());
}