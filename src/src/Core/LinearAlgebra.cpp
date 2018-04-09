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
 * LinearAlgebra.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: mircot
 */

#include "uavAP/Core/protobuf/messages/Positions.pb.h"
#include "uavAP/Core/LinearAlgebra.h"
#include <iostream>

Vector3
toVector(const PositionENU& pos)
{
    return Vector3(pos.east(), pos.north(), pos.up());
}

Vector2
rotate2Drad(Vector2 vec, double rad)
{
    double c = std::cos(rad);
    double s = std::sin(rad);
    double x = vec.x() * c - vec.y() * s;
    double y = vec.x() * s + vec.y() * c;
    return Vector2(x, y);
}

std::istream&
operator>>(std::istream& is, Vector3& obj)
{
    is >> obj[0];
    is >> obj[1];
    is >> obj[2];
    return is;
}

std::ostream&
operator<<(std::ostream& os, const Vector3& obj)
{
    os << obj[0] << " " << obj[1] << " " << obj[2] << " ";
    return os;
}

std::istream&
operator>>(std::istream& is, EigenLine& obj)
{
    Vector3 origin, direction;
    is >> origin;
    is >> direction;
    obj = EigenLine(origin, direction);
    return is;
}

std::ostream&
operator<<(std::ostream& os, const EigenLine& obj)
{
    os << obj.origin() << obj.direction();
    return os;
}

std::istream&
operator>>(std::istream& is, EigenHyperplane& obj)
{
    Vector3 norm, off;
    is >> norm >> off;
    obj.normal();
    obj.offset();
    obj = EigenHyperplane(norm, off);
    return is;
}

std::ostream&
operator<<(std::ostream& os, const EigenHyperplane& obj)
{
    os << obj.normal() << obj.offset();
    return os;
}

double
headingFromENU(const Vector3& vec)
{
    return atan2(vec.x(), vec.y());
}

double
headingFromENU(const Vector2& vec)
{
    return atan2(vec.x(), vec.y());
}

double
boundAngleRad(double angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
